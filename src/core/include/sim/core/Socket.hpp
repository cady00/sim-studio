#pragma once

// Berkeley sockets on Linux, Winsock2 on Windows. Callers keep one code path.

#include <cstddef>
#include <cstdint>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace sim {

#ifdef _WIN32
using SocketFd = SOCKET;
inline constexpr SocketFd kInvalidSocket = INVALID_SOCKET;
using SocketCount = int;
#else
using SocketFd = int;
inline constexpr SocketFd kInvalidSocket = -1;
using SocketCount = ssize_t;
#endif

inline void startupSockets() {
#ifdef _WIN32
    static const int started = [] {
        WSADATA data{};
        return ::WSAStartup(MAKEWORD(2, 2), &data);
    }();
    (void)started;
#endif
}

inline bool socketOk(SocketFd fd) {
#ifdef _WIN32
    return fd != INVALID_SOCKET;
#else
    return fd >= 0;
#endif
}

inline void closeSocket(SocketFd fd) {
#ifdef _WIN32
    ::closesocket(fd);
#else
    ::close(fd);
#endif
}

inline int lastSocketError() {
#ifdef _WIN32
    return ::WSAGetLastError();
#else
    return errno;
#endif
}

inline bool socketInProgress(int error) {
#ifdef _WIN32
    return error == WSAEWOULDBLOCK || error == WSAEINPROGRESS;
#else
    return error == EINPROGRESS;
#endif
}

inline bool setReuseAddress(SocketFd fd) {
#ifdef _WIN32
    const char reuse = 1;
    return ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) == 0;
#else
    const int reuse = 1;
    return ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) == 0;
#endif
}

inline bool setBlocking(SocketFd fd, bool blocking) {
#ifdef _WIN32
    u_long mode = blocking ? 0u : 1u;
    return ::ioctlsocket(fd, FIONBIO, &mode) == 0;
#else
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    const int next = blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);
    return ::fcntl(fd, F_SETFL, next) == 0;
#endif
}

inline int pollSocket(SocketFd fd, bool write, int timeoutMs) {
#ifdef _WIN32
    WSAPOLLFD wait{};
    wait.fd = fd;
    wait.events = write ? POLLOUT : POLLIN;
    const int ready = ::WSAPoll(&wait, 1, timeoutMs);
    if (ready > 0 && (wait.revents & (POLLERR | POLLHUP)) != 0) {
        return -1;
    }
    return ready;
#else
    pollfd wait{};
    wait.fd = fd;
    wait.events = write ? POLLOUT : POLLIN;
    const int ready = ::poll(&wait, 1, timeoutMs);
    if (ready > 0 && (wait.revents & (POLLERR | POLLHUP)) != 0) {
        return -1;
    }
    return ready;
#endif
}

inline int socketSoError(SocketFd fd) {
    int soError = 0;
#ifdef _WIN32
    int length = sizeof(soError);
    if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&soError), &length) != 0) {
        return lastSocketError();
    }
#else
    socklen_t length = sizeof(soError);
    if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soError, &length) != 0) {
        return lastSocketError();
    }
#endif
    return soError;
}

inline SocketCount socketSend(SocketFd fd, const void* data, std::size_t size) {
#ifdef _WIN32
    return ::send(fd, static_cast<const char*>(data), static_cast<int>(size), 0);
#else
    return ::send(fd, data, size, MSG_NOSIGNAL);
#endif
}

inline SocketCount socketRecv(SocketFd fd, void* data, std::size_t size) {
#ifdef _WIN32
    return ::recv(fd, static_cast<char*>(data), static_cast<int>(size), 0);
#else
    return ::recv(fd, data, size, 0);
#endif
}

}  // namespace sim
