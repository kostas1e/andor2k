#include "andor2k.hpp"
#include "andor2kd.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

/// @brief Resolve an image command-string
/// An image command-string always starts with the string "image" and then is
/// followed by a list of options which can be:
/// * --nimages [INT]
///     if [INT] > 1 then the function will set the Acquisition Mode to Run Till
///     Abort. If [INT] == 1, then the function will set the Acquisition Mode to
///     Single Scan.
/// * --[vh]bin [INT] vertical or horizontal binning
/// * --bin [INT] Sets both vertical and horizontal binning
/// * --[vh]start [INT] starting pixel (inclusive, aka starting from 1 not 0)
/// * --[vh]end [INT] ending pixel (inclusive, aka in range [1, 2048])
/// * --filename [STRING] generic filename for exposure(s); a date-string, an
///     index and the extension .fits will be added to the filename(s) when
///     acquiring the exposures.
/// * --type [STRING] can be any of "flat", "bias", "object"
/// * --exposure [FLOAT] exposure time in seconds
/// * --ar-tries [INT] number of tries to access Aristarchos headers (0 means
///     do not try at all)
/// * --object [STRING] Name of object; this will be writeen (as is) in the
///     FITS file header
/// * --filter [STRING] Name of filter; this will be writeen (as is) in the
///     FITS file header
///
/// @param[in] command A c-string holding the command to resolve; the string
///                    should start with the "image" token and hold as many
///                    options as needed. Here are examples of valid commands:
/// "image --bin 2"
/// "image --vbin 2 --hbin 4 --filename foobar --type object --exposure 4.6"
/// @param[out] params The parameters instance where the resolved options (as
///                    extracted from the command) are going to be saved. E.g.
///                    given "image --bin 2" the params variables from vertical
///                    and horizontal binning are goinf to be set to 2
/// @return Anything other than 0 denotes an error
int resolve_image_parameters(const char *command,
                             AndorParameters &params) noexcept {

#ifdef DEBUG
  printf(">> Resolving command: [%s]\n", command);
#endif

  // first string should be "image"
  if (std::strncmp(command, "image", 5))
    return 1;

  if (params.read_out_mode_ != ReadOutMode::Image) {
    printf("-----> WTF at start readoutmode is different!\n");
    return 4;
  }

  // datetime string buffer
  char buf[32];

  // copy the input string so that we can tokenize it
  char string[MAX_SOCKET_BUFFER_SIZE];
  std::memcpy(string, command, sizeof(char) * MAX_SOCKET_BUFFER_SIZE);

  char *token = std::strtok(string, " "); // this is the command (aka image)
  token = std::strtok(nullptr, " ");
  char *end;
  // split remaining substring to tokens and process one at a time
  while (token) {

    /* NUM IMAGES
     *--------------------------------------------------------*/
    if (!std::strncmp(token, "--nimages", 9)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a numeric argument to \"--nimages\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      params.num_images_ = std::strtol(token, &end, 10);
      if (end == token || params.num_images_ < 1) {
        fprintf(stderr,
                "[ERROR][%s] Failed to convert parameter \"%s\" to (valid) "
                "integral numeric value (traceback: %s)\n",
                date_str(buf), token, __func__);
        return 1;
      }
      if (params.num_images_ == 1) {
        params.acquisition_mode_ = AcquisitionMode::SingleScan;
      } else if (params.num_images_ > 1 &&
                 params.acquisition_mode_ == AcquisitionMode::SingleScan) {
        params.acquisition_mode_ = AcquisitionMode::RunTillAbort;
      }

      /* BINNING OPTIONS
       * -------------------------------------------------------*/
    } else if (!std::strncmp(token, "--bin", 5)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a numeric argument to \"--bin\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      params.image_vbin_ = std::strtol(token, &end, 10);
      params.image_hbin_ = params.image_vbin_;
      if (end == token) {
        fprintf(stderr,
                "[ERROR][%s] Failed to convert parameter \"%s\" to integral "
                "numeric value (traceback: %s)\n",
                date_str(buf), token, __func__);
        return 1;
      }
    } else if (!std::strncmp(token, "--hbin", 6)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a numeric argument to \"--hbin\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      params.image_hbin_ = std::strtol(token, &end, 10);
      if (end == token) {
        fprintf(stderr,
                "[ERROR][%s] Failed to convert parameter \"%s\" to integral "
                "numeric value (traceback: %s)\n",
                date_str(buf), token, __func__);
        return 1;
      }
    } else if (!std::strncmp(token, "--vbin", 6)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a numeric argument to \"--vbin\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      params.image_vbin_ = std::strtol(token, &end, 10);
      if (end == token) {
        fprintf(stderr,
                "[ERROR][%s] Failed to convert parameter \"%s\" to integral "
                "numeric value (traceback: %s)\n",
                date_str(buf), token, __func__);
        return 1;
      }

      /* IMAGE DIMENSIONS OPTIONS
       * ----------------------------------------------*/
    } else if (!std::strncmp(token, "--hstart", 8)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a numeric argument to \"--hstart\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      params.image_hstart_ = std::strtol(token, &end, 10);
      if (end == token) {
        fprintf(stderr,
                "[ERROR][%s] Failed to convert parameter \"%s\" to integral "
                "numeric value (traceback: %s)\n",
                date_str(buf), token, __func__);
        return 1;
      }
    } else if (!std::strncmp(token, "--hend", 6)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a numeric argument to \"--hend\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      params.image_hend_ = std::strtol(token, &end, 10);
      if (end == token) {
        fprintf(stderr,
                "[ERROR][%s] Failed to convert parameter \"%s\" to integral "
                "numeric value (traceback: %s)\n",
                date_str(buf), token, __func__);
        return 1;
      }
    } else if (!std::strncmp(token, "--vstart", 8)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a numeric argument to \"--vstart\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      params.image_vstart_ = std::strtol(token, &end, 10);
      if (end == token) {
        fprintf(stderr,
                "[ERROR][%s] Failed to convert parameter \"%s\" to integral "
                "numeric value (traceback: %s)\n",
                date_str(buf), token, __func__);
        return 1;
      }
    } else if (!std::strncmp(token, "--vend", 6)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a numeric argument to \"--vend\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      params.image_vend_ = std::strtol(token, &end, 10);
      if (end == token) {
        fprintf(stderr,
                "[ERROR][%s] Failed to convert parameter \"%s\" to integral "
                "numeric value (traceback: %s)\n",
                date_str(buf), token, __func__);
        return 1;
      }
      if (params.read_out_mode_ != ReadOutMode::Image) {
        printf("-----> WTF at end readoutmode is different! a if\n");
        return 4;
      }

      /* IMAGE FILENAME
       * --------------------------------------------------------*/
    } else if (!std::strncmp(token, "--filename", 10)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a string argument to \"--filename\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      if (std::strlen(token) >= 128) {
        fprintf(stderr,
                "[ERROR][%s] Invalid argument for \"%s\" (traceback: %s)\n",
                date_str(buf), token, __func__);
        return 1;
      }
      std::memset(params.image_filename_, '\0', MAX_FITS_FILENAME_SIZE);
      std::strcpy(params.image_filename_, token);
      if (params.read_out_mode_ != ReadOutMode::Image) {
        printf("-----> WTF at end readoutmode is different! a it\n");
        return 4;
      }

      /* IMAGE TYPE
       * --------------------------------------------------------*/
    } else if (!std::strncmp(token, "--type", 6)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a string argument to \"--type\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      if (std::strlen(token) >= MAX_IMAGE_TYPE_CHARS) {
        fprintf(stderr,
                "[ERROR][%s] Invalid argument for \"%s\" (traceback: %s)\n",
                date_str(buf), token, __func__);
        return 1;
      }
      std::memset(params.type_, '\0', MAX_IMAGE_TYPE_CHARS);
      std::strcpy(params.type_, token);

      if (params.read_out_mode_ != ReadOutMode::Image) {
        printf("-----> WTF at end readoutmode is different! a ot\n");
        return 4;
      }
      /* OBJECT TYPE
       * --------------------------------------------------------*/
    } else if (!std::strncmp(token, "--object", 8)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a string argument to \"--object\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      if (std::strlen(token) >= MAX_OBJECT_NAME_CHARS) {
        fprintf(stderr,
                "[ERROR][%s] Invalid argument for \"%s\" (traceback: %s)\n",
                date_str(buf), token, __func__);
        return 1;
      }
      std::memset(params.object_name_, '\0', MAX_OBJECT_NAME_CHARS);
      std::strcpy(params.object_name_, token);

      if (params.read_out_mode_ != ReadOutMode::Image) {
        printf("-----> WTF at end readoutmode is different! a fn\n");
        return 4;
      }

      /* FILTER NAME
       * --------------------------------------------------------*/
    } else if (!std::strncmp(token, "--filter", 8)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a string argument to \"--filter\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      if (std::strlen(token) >= MAX_FILTER_NAME_CHARS) {
        fprintf(stderr,
                "[ERROR][%s] Invalid argument for \"%s\" (traceback: %s)\n",
                date_str(buf), token, __func__);
        return 1;
      }
      std::memset(params.filter_name_, '\0', MAX_FILTER_NAME_CHARS);
      std::strcpy(params.filter_name_, token);

      if (params.read_out_mode_ != ReadOutMode::Image) {
        printf("-----> WTF at end readoutmode is different! a e\n");
        return 4;
      }
      /* EXPOSURE
       * --------------------------------------------------------*/
    } else if (!std::strncmp(token, "--exposure", 10)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide a float argument to \"--exposure\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      params.exposure_ = std::strtod(token, &end);
      if (end == token || !(params.exposure_ >= 0e0)) {
        fprintf(
            stderr,
            "[ERROR][%s] Failed to convert parameter \"%s\" to a (valid) float "
            "numeric value (traceback: %s)\n",
            date_str(buf), token, __func__);
        return 1;
      }

      /* ARISTARCHOS HEADERS
       * --------------------------------------------------------*/
    } else if (!std::strncmp(token, "--ar-tries", 10)) {
      if (token = std::strtok(nullptr, " "); token == nullptr) {
        fprintf(stderr,
                "[ERROR][%s] Must provide an int argument to \"--ar-tries\" "
                "(traceback: %s)\n",
                date_str(buf), __func__);
        return 1;
      }
      params.ar_hdr_tries_ = std::strtol(token, &end, 10);
      if (end == token) {
        fprintf(
            stderr,
            "[ERROR][%s] Failed to convert parameter \"%s\" to a (valid) int "
            "numeric value (traceback: %s)\n",
            date_str(buf), token, __func__);
        return 1;
      }

    } else {
      fprintf(stderr,
              "[WRNNG][%s] Ignoring input parameter \"%s\" (traceback: %s)\n",
              date_str(buf), token, __func__);
    }

    token = std::strtok(nullptr, " "); // resolve next token (if any)
  }

  // some final testing
  int status = 0;
  if (!params.image_vbin_ || !params.image_hbin_) {
    fprintf(stderr,
            "[ERROR][%s] Binning parameter(s) cannot be zero! Smallest value "
            "is 1 (traceback: %s)\n",
            date_str(buf), __func__);
    status = 2;
  }
  if ((params.image_hstart_ < 1 || params.image_hstart_ >= MAX_PIXELS_IN_DIM) ||
      (params.image_vstart_ < 1 || params.image_vstart_ >= MAX_PIXELS_IN_DIM)) {
    fprintf(
        stderr,
        "[ERROR][%s] Starting pixel must be in range [1, %d) (traceback: %s)\n",
        date_str(buf), MAX_PIXELS_IN_DIM, __func__);
    status = 2;
  }
  if ((params.image_hend_ <= 1 || params.image_hend_ > MAX_PIXELS_IN_DIM) ||
      (params.image_vend_ <= 1 || params.image_vend_ > MAX_PIXELS_IN_DIM)) {
    fprintf(
        stderr,
        "[ERROR][%s] Ending pixel must be in range [2, %d] (traceback: %s)\n",
        date_str(buf), MAX_PIXELS_IN_DIM, __func__);
    status = 2;
  }
  if (params.exposure_ < 0e0) {
    fprintf(
        stderr,
        "[ERROR][%s] Exposure must be a positive real number (traceback: %s)\n",
        date_str(buf), __func__);
    status = 2;
  }

#ifndef MOCK_CAMERA
  // TODO why the fuck do i need this here? i actually think i don't, its fixed
  std::memset(params.save_dir_, 0, 128);
  std::strcpy(params.save_dir_, "/home/andor2k/fits");
#endif

  if (params.read_out_mode_ != ReadOutMode::Image) {
    fprintf(stderr,
            "[ERROR][%s] ReadOutMode can only be \'Image\', for some reason it "
            "is not! (traceback: %s)\n",
            date_str(buf), __func__);
    return 4;
  }

  return status;
}
