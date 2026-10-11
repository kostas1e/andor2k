#include "cpp_socket.hpp"
#include <cassert>
#include <cstring>
#include <stdexcept>
#ifdef SOCKET_LOGGER
#include <ctime>
#endif

#ifdef SOCKET_LOGGER
andor2k::SocketLogger::SocketLogger(const char *fn)
    : m_logger(fn, std::ios_base::app) {
  if (!m_logger.is_open()) {
    throw std::runtime_error("[ERROR] Failed to open logger");
  }
}

andor2k::SocketLogger::~SocketLogger() noexcept { m_logger.close(); }

void andor2k::SocketLogger::print_msg(int sockid, const char *msg,
                                      int info_type) {
  assert(m_logger.is_open());
  std::string info_str;
  switch (info_type) {
  case 0:
    info_str = std::string("ERROR  ");
    break;
  case 1:
    info_str = std::string("WARNING");
    break;
  default:
    info_str = std::string("DEBUG  ");
    break;
  }
  std::time_t now = std::time(nullptr);
  std::strftime(tm_buf, sizeof(tm_buf), "%Y-%b-%dT%H:%M:%S",
                std::localtime(&now));
  m_logger << "\n[" << tm_buf << "] SockId: " << sockid << " : " << info_str
           << msg;
  assert(m_logger);
}
#endif

#ifdef SOCKET_LOGGER
andor2k::Socket::Socket(SocketLogger *logger) noexcept
    : m_sockid(socket(AF_INET, SOCK_STREAM, 0)), m_logger(logger) {
  if (m_sockid != -1)
    std::memset(&m_address, 0, sizeof(m_address));
  m_logger->print_msg(m_sockid, " Creating new Socket", 10);
}
#else
andor2k::Socket::Socket() noexcept : m_sockid(socket(AF_INET, SOCK_STREAM, 0)) {
  if (m_sockid != -1)
    std::memset(&m_address, 0, sizeof(m_address));
}
#endif

#ifdef SOCKET_LOGGER
andor2k::Socket::Socket(int sockid, sockaddr_in addr,
                        SocketLogger *logger) noexcept
    : m_sockid(sockid), m_logger(logger)
#else
andor2k::Socket::Socket(int sockid, sockaddr_in addr) noexcept
    : m_sockid(sockid)
#endif
{
  std::memcpy(&m_address, &addr, sizeof(addr));
}

andor2k::Socket::~Socket() noexcept {
#ifdef SOCKET_LOGGER
  m_logger->print_msg(m_sockid, " Closing Socket", 10);
#endif
  close(m_sockid);
}

void andor2k::Socket::set_sock_addr(int port, const char *ip) noexcept {
  // m_address.sin_addr.s_addr = ip? (inet_addr(ip)) : (htonl(INADDR_ANY));
  if (ip && !(std::strcmp("localhost", ip))) {
    m_address.sin_addr.s_addr = inet_addr("127.0.0.1");
  } else if (ip) {
    m_address.sin_addr.s_addr = inet_addr(ip);
  } else {
    m_address.sin_addr.s_addr = htonl(INADDR_ANY);
  }
  m_address.sin_family = AF_INET;
  m_address.sin_port = htons(port);
}

int andor2k::Socket::send(const char *msg, int flag) const noexcept {
#ifdef SOCKET_LOGGER
  m_logger->print_msg(m_sockid, " Sending msg via Socket", 10);
#endif
  // MSG_NOSIGNAL: suppress SIGPIPE if the remote end has closed the connection;
  // return -1/EPIPE instead so callers can handle the error without dying.
  return ::send(m_sockid, msg, std::strlen(msg), flag | MSG_NOSIGNAL);
}

int andor2k::Socket::recv(char *buffer, std::size_t buf_sz,
                          int flag) const noexcept {
#ifdef SOCKET_LOGGER
  m_logger->print_msg(m_sockid, " Receiving msg via Socket", 10);
#endif
  return ::recv(m_sockid, buffer, buf_sz, flag);
}

int andor2k::Socket::bind(int port) noexcept {
  set_sock_addr(port);
#ifdef SOCKET_LOGGER
  m_logger->print_msg(m_sockid, " Binding Socket to port", 10);
#endif
  return ::bind(m_sockid, (struct sockaddr *)&m_address, sizeof(m_address));
}

int andor2k::Socket::listen(int max_connections) const noexcept {
#ifdef SOCKET_LOGGER
  m_logger->print_msg(m_sockid, " Setting Socket to listen mode", 10);
#endif
  int maxcon = max_connections < 0 ? this->maxconnections() : max_connections;
  return ::listen(m_sockid, maxcon);
}

andor2k::Socket andor2k::Socket::accept(int &status) noexcept {
  struct sockaddr_in client_sock_addr;
  int len = sizeof(client_sock_addr);
  int client_sock_id = ::accept(m_sockid, (struct sockaddr *)&client_sock_addr,
                                (socklen_t *)&len);
  status = client_sock_id;
#ifdef SOCKET_LOGGER
  m_logger->print_msg(m_sockid,
                      std::string(" Accepting new Socket with id #") +
                          std::to_string(client_sock_id),
                      10);
  return Socket(client_sock_id, client_sock_addr, m_logger);
#else
  return Socket(client_sock_id, client_sock_addr);
#endif
}

int andor2k::Socket::connect(const char *ip, int port) noexcept {
  /*
  int status = 0;
  if ((status = inet_pton(AF_INET, ip, &m_address.sin_addr)) <= 0)
    return status;
  */
  this->set_sock_addr(port, ip);
#ifdef SOCKET_LOGGER
  m_logger->print_msg(m_sockid, " Connecting Socket", 10);
#endif
  return ::connect(m_sockid, (struct sockaddr *)&m_address, sizeof(m_address));
}

int andor2k::Socket::set_non_blocking() noexcept {
  int flags;
  if ((flags = fcntl(m_sockid, F_GETFL)) < 0)
    return flags;
  return fcntl(m_sockid, F_SETFL, flags | O_NONBLOCK);
}

andor2k::ClientSocket::ClientSocket(const char *host, int port
#ifdef SOCKET_LOGGER
                                    ,
                                    andor2k::SocketLogger *logger
#endif
                                    )
    : m_socket(
#ifdef SOCKET_LOGGER
          logger
#endif
      ) {
  int status;
  // check the socket's file descriptor; should be >=0
  if ((status = m_socket.sockid()) < 0) {
    // perror("[ERROR] Failed to construct socket!");
    status = -10;
  }

  // connect (this) client to server socket
  if (status >= 0) {
    if ((status = m_socket.connect(host, port)) < 0) {
      // perror("[ERROR] Socket failed to connect!");
      status = -30;
    }
  }

  // if something has gone wrong, throw!
  if (status < 0) {
    std::string host_str(host), port_str(std::to_string(port)),
        errno_str(std::to_string(errno)), status_str(std::to_string(status));
    std::string err_msg = "[ERROR] Failed to create ClientSocket for " +
                          host_str + ":" + port_str +
                          " (errno/status: " + errno_str + "/" + status_str;
    errno = 0;
    throw std::runtime_error(err_msg);
  }
  // all done
}

andor2k::ServerSocket::ServerSocket(int port
#ifdef SOCKET_LOGGER
                                    ,
                                    andor2k::SocketLogger *logger
#endif
                                    )
    : m_socket(
#ifdef SOCKET_LOGGER
          logger
#endif
      ) {
  int status;
  // check the socket's file descriptor; should be >=0
  if ((status = m_socket.sockid()) < 0) {
    // perror("[ERROR] Failed to construct socket!");
    status = -10;
  }

  // assign port
  m_socket.set_sock_addr(port);

  // bind the socket
  if (status >= 0) {
    if ((status = m_socket.bind(port)) < 0) {
      // perror("[ERROR] Failed to bind socket!");
      status = -30;
    }
  }

  // now server is ready to listen
  if (status >= 0) {
    if ((status = m_socket.listen()) < 0) {
      // perror("[ERROR] Failed to set listen state!");
      status = -40;
    }
  }

  // if something has gone wrong, throw!
  if (status < 0) {
    std::string port_str(std::to_string(port)),
        errno_str(std::to_string(errno)), status_str(std::to_string(status));
    std::string err_msg =
        std::string("[ERROR] Failed to create ServerSocket ") +
        "at port :" + port_str + " (errno/status: " + errno_str + "/" +
        status_str;
    errno = 0;
    throw std::runtime_error(err_msg);
  }
  // all done
}
