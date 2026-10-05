#pragma once

#include <cerrno>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <csignal>
#include <unistd.h>
#endif

namespace sim {

inline long currentProcessId() {
#ifdef _WIN32
    return static_cast<long>(::GetCurrentProcessId());
#else
    return static_cast<long>(::getpid());
#endif
}

// True only when the pid is known to have exited. A live process, or one this
// process is not allowed to query, stays false so a foreign session is left alone.
inline bool processHasExited(long pid) {
    if (pid <= 0) {
        return false;
    }
#ifdef _WIN32
    HANDLE handle = ::OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, static_cast<DWORD>(pid));
    if (handle == nullptr) {
        return ::GetLastError() == ERROR_INVALID_PARAMETER;
    }
    DWORD code = STILL_ACTIVE;
    const BOOL ok = ::GetExitCodeProcess(handle, &code);
    ::CloseHandle(handle);
    return ok == TRUE && code != STILL_ACTIVE;
#else
    if (::kill(static_cast<pid_t>(pid), 0) == 0) {
        return false;
    }
    return errno == ESRCH;
#endif
}

}  // namespace sim
