#include "andor2kd.hpp"

#include "andor2k.hpp"
#include "atmcdLXd.h"
#include "cpp_socket.hpp"
#include "cppfits.hpp"
#include "DualWheelMapper.hpp"
#include "fits_header.hpp"
#include "IFilterWheel.hpp"
#ifdef MOCK_WHEEL
#include "MockFilterWheel.hpp"
#else
#include "MaxonFilterWheel.hpp"
#endif

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#ifdef MOCK_CAMERA
#include <filesystem>
#endif
#include <fstream>
#include <pthread.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

using andor2k::ServerSocket;
using andor2k::Socket;
using namespace std::chrono_literals;

// Global constants for abort/interrupt
extern int sig_abort_set;
extern int sig_interrupt_set;
extern int abort_exposure_set;

// Global filter wheel instance - driver selected at compile time:
//   --enable-mock-wheel -> MockFilterWheel (no hardware)
//   (default)           -> MaxonFilterWheel (EPOS4 via USB)
andor2k::IFilterWheel *g_wheel = nullptr;

// buffers and constants for socket communication
constexpr int INTITIALIZE_TO_TEMP = -50;
char fits_file[MAX_FITS_FILE_SIZE] = {'\0'};
char now_str[32] = {'\0'}; // YYYY-MM-DD HH:MM:SS
char buffer[MAX_SOCKET_BUFFER_SIZE];

/// Signal handler to kill daemon (calls shutdown() and then exits)
void kill_daemon(int signal) noexcept {
  printf(
      "[DEBUG][%s] Caught signal (#%d); shutting down daemon (traceback: %s)\n",
      date_str(now_str), signal, __func__);
  if (g_wheel) {
    g_wheel->Halt();
    g_wheel->Disconnect();
    delete g_wheel;
    g_wheel = nullptr;
  }
  system_shutdown(); // RUN
  printf("[DEBUG][%s] Goodbye!\n", date_str(now_str));
  exit(signal);
}

/// Signal handler for SEGFAULT (calls shutdown() and then exits)
void segfault_sigaction(int signal, siginfo_t *si, void *arg) {
  printf("[FATAL][%s] Caught segfault at address %p; shutting down daemon "
         "(traceback: %s)\n",
         date_str(now_str), si->si_addr, __func__);
  if (g_wheel) {
    g_wheel->Halt();
    delete g_wheel;
    g_wheel = nullptr;
  }
  system_shutdown(); // RUN
  printf("[DEBUG][%s] Goodbye!\n", date_str(now_str));
  exit(signal);
}

/// Set ANDOR2K temperature via a command of type: "settemp [ITEMP]"
/// @param[in] command A (char) buffer holding the command to be executed (null
///            terminated). The command should be a c-string of type:
///            "settemp [ITEMP]" where ITEMP is an integer denoting the
///            temperature to be reached by the ANDOR2K camera
int set_temperature(const char *command, const Socket &socket) noexcept {
  errno = 0;
  char buffer[MAX_SOCKET_BUFFER_SIZE];
  if (std::strncmp(command, "settemp", 7)) {
    socket_sprintf(socket, buffer, "done;error:1;status:Invalid command!");
    return 1;
  }

  // we expect that after the temperatues, we have a valid int
  char *end;
  int target_temp = std::strtol(command + 7, &end, 10);
  if (end == command + 7 || errno) {
    errno = 0;
    fprintf(
        stderr,
        "[ERROR][%s] Failed to resolve target temperature in command \"%s\"\n",
        date_str(now_str), command);
    fprintf(stderr, "[ERROR][%s] Skippig command \"%s\"\n", now_str, command);
    socket_sprintf(socket, buffer, "done;error:1;status:Invalid command!");
    return 1;
  }

  if (target_temp < ANDOR_MIN_TEMP || target_temp > ANDOR_MAX_TEMP) {
    fprintf(stderr,
            "[ERROR][%s] Refusing to set temperature outside limits: [%+3d, "
            "%+3d]\n",
            date_str(now_str), ANDOR_MIN_TEMP, ANDOR_MAX_TEMP);
    fprintf(stderr, "[ERROR][%s] Skippig command \"%s\"\n", now_str, command);
    socket_sprintf(socket, buffer, "done;error:1;status:Invalid command!");
    return 1;
  }

  // command seems ok .... do it!
  return cool_to_temperature(target_temp, &socket);
}

int get_image(const char *command, const Socket &socket,
              AndorParameters &params) noexcept {

  // first try to resolve the image parameters of the command
  if (resolve_image_parameters(command, params)) {
    fprintf(stderr,
            "[ERROR][%s] Failed to resolve image parameters; aborting request! "
            "(traceback: %s)\n",
            date_str(now_str), __func__);
    return 1;
  }

  // setup the acquisition process for the image(s); also prepare FITS headers
  // for later use in the file(s) to be saved
  int width, height;
  float vsspeed, hsspeed;
  FitsHeaders fheaders;
  at_32 *data = nullptr; // remember to free this
  int status = 0;
  if (setup_acquisition(&params, &fheaders, width, height, vsspeed, hsspeed,
                        data)) {
    fprintf(stderr,
            "[ERROR][%s] Failed to setup acquisition; aborting request! "
            "(traceback: %s)\n",
            date_str(now_str), __func__);
    status = 2;
  }

  if (!status) {
    if (status =
            get_acquisition(&params, &fheaders, width, height, data, socket);
        status != 0) {
      fprintf(stderr,
              "[ERROR][%s] Failed to get/save image(s); aborting request now "
              "(traceback: %s)\n",
              date_str(now_str), __func__);
      status = 3;
    }
  }

  // free memory and return
  delete[] data;
  return status;
}

int set_param_value(const char *command, AndorParameters &params) noexcept {
  if (std::strncmp(command, "setparam", 8))
    return 1;

  // copy the input string so that we can tokenize it
  char string[MAX_SOCKET_BUFFER_SIZE];
  std::memcpy(string, command, MAX_SOCKET_BUFFER_SIZE);

  int ival;
  float fval;

  char *token = std::strtok(string, " "); // this is the command (aka setparam)
  token = std::strtok(nullptr, " ");
  char *start, *end;
  // split remaining substring to tokens and process one at a time
  while (token) {

    if (!std::strncmp(token, "acqmode=", 8)) {
      ival = std::strtol(token + 8, &end, 10);
      if (end == token) {
        return 10;
      }
      params.acquisition_mode_ = static_cast<AcquisitionMode>(ival);
      printf("[DEBUG][%s] Changing Acquisition Mode to : %d!\n",
             date_str(now_str), ival);

    } else if (!std::strncmp(token, "kineticcycletime=", 17)) {
      fval = std::strtod(token + 17, &end);
      if (end == token) {
        return 10;
      }
      params.kinetics_cycle_time_ = fval;
      printf("[DEBUG][%s] Changing Kinetic Cycle Time to : %.3fsec!\n",
             date_str(now_str), fval);
    } else if (!std::strncmp(token, "observername=", 13)) {
      std::strcpy(params.observer_name_, token + 13);
      params.observer_name_[31] = '\0';
    } else if (!std::strncmp(token, "hsspeed=", 8)) {
      ival = std::strtod(token + 8, &end);
      if (end == token) {
        return 10;
      }
      // let's ask the cammera to match the index to a current MHz value
      {
        int num_speeds;
        float speed;
        if (GetNumberHSSpeeds(0, 0, &num_speeds) != DRV_SUCCESS) {
          fprintf(stderr,
                  "[WRNNG][%s] Failed to get number of available horizontal "
                  "speeds for camera! (command: [%s])\n",
                  date_str(now_str), token);
          return 2;
        }
        if ((ival < 0 || ival >= num_speeds) ||
            (GetHSSpeed(0, 0, ival, &speed) != DRV_SUCCESS)) {
          fprintf(stderr,
                  "[WRNNG][%s] Index is out of limits for horizontal speed ! "
                  "(command: [%s])\n",
                  date_str(now_str), token);
          return 2;
        }
        printf("[DEBUG][%s] Changing horizontal shift speed to : %.3fMHz!\n",
               date_str(now_str), speed);
      }
      params.hsspeed = ival;
    } else if (!std::strncmp(token, "preampgain=", 11)) {
      ival = std::strtod(token + 11, &end);
      if ((end == token) || (ival < 0 || ival >= 3)) {
        fprintf(stderr,
                "[WRNNG][%s] Index is out of limits for pre-amp gain! "
                "(command: [%s])\n",
                date_str(now_str), token);
        return 11;
      }
      {
        float fac;
        if (GetPreAmpGain(ival, &fac) != DRV_SUCCESS) {
          fprintf(stderr,
                  "[ERROR][%s] Failed retrieving Pre-Amp Gain; wierd ..."
                  "(traceback: %s)\n",
                  date_str(now_str), __func__);
          return 1;
        }
        printf("[DEBUG][%s] Changing Pre-Amp Gain factor to : %.1fx\n",
               date_str(now_str), fac);
      }
      params.preampgain = ival;
    } else {
      fprintf(stderr,
              "[WRNNG][%s] Skipping token in paramter set command: [%s]\n",
              date_str(now_str), token);
    }

    token = std::strtok(nullptr, " "); // resolve next token (if any)
  }

  return 0;
}

int handle_wheel_command(const char *command, const Socket &socket) noexcept {
  char buf[MAX_SOCKET_BUFFER_SIZE];

  // "wheel status" - most frequent; check first
  if (!std::strncmp(command, "wheel status", 12)) {
    int fault = g_wheel->GetFaultState();
    socket_sprintf(socket, buf,
                   "fconnected:1;fmoving:%d;fpos:%d;"
                   "fupper_qc:%ld;flower_qc:%ld;fhomed:%d;"
                   "fupper_fault:%d;flower_fault:%d",
                   g_wheel->IsMoving() ? 1 : 0, g_wheel->GetPosition(),
                   g_wheel->GetUpperQC(), g_wheel->GetLowerQC(),
                   g_wheel->IsHomed() ? 1 : 0, (fault & 1) ? 1 : 0,
                   (fault & 2) ? 1 : 0);
    return 0;
  }

  if (!std::strncmp(command, "wheel home", 10)) {
    int err = g_wheel->GoHome();
    if (err == 0) {
      // fmove_qc:0 tells the client to keep the indeterminate bar
      // (homing distance is not known in advance).
      socket_sprintf(socket, buf, "fmove_qc:0;");
      auto deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(60);
      int tick = 0;
      while (g_wheel->IsMoving()) {
        if (std::chrono::steady_clock::now() > deadline) {
          g_wheel->Halt();
          socket_sprintf(socket, buf, "done;error:10;status:timeout;");
          return 10;
        }
        // Non-blocking peek: client may send "wheel halt" mid-move.
        {
          char rbuf[32] = {0};
          if (socket.recv(rbuf, sizeof(rbuf) - 1, MSG_DONTWAIT) > 0 &&
              std::strncmp(rbuf, "wheel halt", 10) == 0)
            g_wheel->Halt();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Every 200 ms send an intermediate QC update for the GUI.
        if (++tick % 2 == 0) {
          socket_sprintf(socket, buf, "fupper_qc:%ld;flower_qc:%ld;",
                         g_wheel->GetUpperQC(), g_wheel->GetLowerQC());
        }
      }
    }
    socket_sprintf(socket, buf, "done;error:%d;", err);
    return err;
  }

  if (!std::strncmp(command, "wheel halt", 10)) {
    int err = g_wheel->Halt();
    socket_sprintf(socket, buf, "done;error:%d;", err);
    return err;
  }

  if (!std::strncmp(command, "wheel setpos", 12)) {
    char *end;
    int slot = static_cast<int>(std::strtol(command + 12, &end, 10));
    if (end == command + 12 || slot < 0 || slot > 10) {
      socket_sprintf(socket, buf, "done;error:1;");
      return 1;
    }
    long upper_before = g_wheel->GetUpperQC();
    long lower_before = g_wheel->GetLowerQC();
    // Compute the planned travel distance BEFORE starting the move so the
    // client can display a real percentage in the progress bar.
    auto phys = andor2k::DualWheelMapper::MapVirtualToPhysical(
        slot, upper_before, lower_before);
    long delta_u = phys.upper_qc - upper_before;
    long delta_l = phys.lower_qc - lower_before;
    long move_qc = std::max(delta_u, delta_l);
    int err = g_wheel->SetPosition(slot);
    if (err == 0) {
      // First streaming line: total QC for this move (0 -> client shows
      // indeterminate bar; > 0 -> client shows a real percentage).
      socket_sprintf(socket, buf, "fmove_qc:%ld;", move_qc);
      auto deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(60);
      int tick = 0;
      while (g_wheel->IsMoving()) {
        if (std::chrono::steady_clock::now() > deadline) {
          g_wheel->Halt();
          socket_sprintf(socket, buf, "done;error:10;status:timeout;");
          return 10;
        }
        // Non-blocking peek: client may send "wheel halt" mid-move.
        {
          char rbuf[32] = {0};
          if (socket.recv(rbuf, sizeof(rbuf) - 1, MSG_DONTWAIT) > 0 &&
              std::strncmp(rbuf, "wheel halt", 10) == 0)
            g_wheel->Halt();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Every 200 ms send an intermediate QC update for the GUI.
        if (++tick % 2 == 0) {
          socket_sprintf(socket, buf, "fupper_qc:%ld;flower_qc:%ld;",
                         g_wheel->GetUpperQC(), g_wheel->GetLowerQC());
        }
      }
    }
    socket_sprintf(socket, buf, "done;error:%d;", err);
    return err;
  }

  // Unknown wheel sub-command
  fprintf(stderr, "[ERROR][%s] Unknown wheel command: \"%s\"\n",
          date_str(now_str), command);
  socket_sprintf(socket, buf, "done;error:1;");
  return 1;
}

int resolve_command(const char *command, const Socket &socket,
                    AndorParameters &params) noexcept {
  if (!(std::strncmp(command, "settemp", 7))) {
    return set_temperature(command, socket);
  } else if (!(std::strncmp(command, "shutdown", 8))) {
    return -100;
  } else if (!(std::strncmp(command, "status", 6))) {
    // report here and also send to client
    return print_status(socket);
  } else if (!(std::strncmp(command, "setparam", 8))) {
    return set_param_value(command, params);
  } else if (!(std::strncmp(command, "image", 5))) {
    if (g_wheel && g_wheel->IsMoving()) {
      char buf[MAX_SOCKET_BUFFER_SIZE];
      socket_sprintf(socket, buf,
                     "done;error:1;status:Wheel is moving - halt or wait;");
      return 1;
    }
    return get_image(command, socket, params);
  } else if (!(std::strncmp(command, "abort", 5))) {
    abort_exposure_set = true;
    return 0;
  } else if (!(std::strncmp(command, "wheel", 5))) {
    return handle_wheel_command(command, socket);
  } else {
    fprintf(stderr,
            "[ERROR][%s] Failed to resolve command: \"%s\"; doing nothing!\n",
            date_str(now_str), command);
    return 1;
  }
}

int chat(const Socket &socket, AndorParameters &params) {
  int bytes_r;

  for (;;) {

    // read message from client into buffer
    std::memset(buffer, 0, MAX_SOCKET_BUFFER_SIZE);
    bytes_r = socket.recv(buffer, 1024);

    if (bytes_r < 1)
      return 0;

    // perform the operation requested by client
    int answr = resolve_command(buffer, socket, params);
    if (answr == -100) {
      printf(
          "[DEBUG][%s] Received shutdown command; initializing exit sequence\n",
          date_str(now_str));
      break;
    }
  }

  return -1;
}

int main() {
  int sock_status;
  unsigned int error;

  // register signal for SEGFAULT
  struct sigaction sa;
  std::memset(&sa, 0, sizeof(struct sigaction));
  sigemptyset(&sa.sa_mask);
  sa.sa_sigaction = segfault_sigaction;
  sa.sa_flags = SA_SIGINFO;
  sigaction(SIGSEGV, &sa, NULL);

  // register signals and accossiated and signal handlers
  signal(SIGINT, kill_daemon);
  signal(SIGQUIT, kill_daemon);
  signal(SIGTERM, kill_daemon);

  // ANDOR2K parameters controlling usage
  AndorParameters params;

  // initialize AndorParameters
  params.set_defaults();
#ifdef MOCK_CAMERA
  // Ensure the sim FITS output directory exists and point params at it
  constexpr const char *kSimFitsDir = "/tmp/andor2k/fits";
  {
    std::error_code ec;
    std::filesystem::create_directories(kSimFitsDir, ec);
    if (ec)
      fprintf(stderr, "[WRNNG][%s] Could not create %s: %s\n",
              date_str(now_str), kSimFitsDir, ec.message().c_str());
    else
      printf("[DEBUG][%s] SIM: FITS output directory: %s\n", date_str(now_str),
             kSimFitsDir);
  }
  std::strcpy(params.save_dir_, kSimFitsDir);
#endif
  if (params.read_out_mode_ != ReadOutMode::Image) {
    printf("WTF?? main ~199 readoutmode is not image\n");
    return 10;
  }

  // select the camera
  if (select_camera(params.camera_num_) < 0) {
    fprintf(stderr, "[FATAL][%s] Failed to select camera...exiting\n",
            date_str(now_str));
    return 10;
  }

  // report daemon initialization
  printf("[DEBUG][%s] Initializing ANDOR2K daemon service\n",
         date_str(now_str));

  // initialize CCD
  printf("[DEBUG][%s] Initializing CCD ....", date_str(now_str));
  error = Initialize(params.initialization_dir_);
  if (error != DRV_SUCCESS) {
    fprintf(stderr, "[FATAL][%s] Initialisation error...exiting\n",
            date_str(now_str));
    return 10;
  }
  // allow initialization ... go to sleep for two seconds
  std::this_thread::sleep_for(2000ms);
  printf("... ok!\n");

  // Select filter wheel driver based on build configuration
#ifdef MOCK_WHEEL
  g_wheel = new andor2k::MockFilterWheel();
#else
  g_wheel = new andor2k::MaxonFilterWheel();
#endif
  if (g_wheel->Connect() != 0) {
    fprintf(stderr,
            "[FATAL][%s] Filter wheel Connect() failed after retries - "
            "both EPOS4 drives must be present and powered.\n"
            "        Check USB cables, power, and node-ID DIP switches.\n",
            date_str(now_str));
    return 10;
  }
  printf("[DEBUG][%s] Filter wheel ready\n", date_str(now_str));

  // cool down if needed RUN
  if (cool_to_temperature(INTITIALIZE_TO_TEMP)) {
    fprintf(stderr, "[FATAL][%s] Failed to set target temperature...exiting\n",
            date_str(now_str));
    return 10;
  }

  try {
    ServerSocket server_sock(SOCKET_PORT);

    printf("[DEBUG][%s] Listening on port %d\n", date_str(now_str),
           SOCKET_PORT);
    printf("[DEBUG][%s] Service is up and running ... waiting for input\n",
           date_str(now_str));

    int shutdown_received = 0;
    while (shutdown_received >= 0) {
      // creating hearing child socket
      Socket child_socket = server_sock.accept(sock_status);
      if (sock_status < 0) {
        fprintf(stderr,
                "[FATAL][%s] Failed to create child socket ... exiting\n",
                date_str(now_str));
        return 1;
      }
      printf("[DEBUG][%s] Waiting for instructions ...\n", date_str(now_str));

      // communicate with client
      shutdown_received = chat(child_socket, params);
    }

  } catch (std::exception &e) {
    fprintf(stderr, "[ERROR][%s] Failed creating deamon\n", date_str(now_str));
    fprintf(stderr, "[FATAL][%s] ... exiting\n", date_str(now_str));
  }

  // shutdown system
  if (g_wheel) {
    g_wheel->Halt();
    g_wheel->Disconnect();
    delete g_wheel;
    g_wheel = nullptr;
  }
  system_shutdown();

  return 0;
}
