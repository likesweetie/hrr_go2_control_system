#include "daemon/daemon.hpp"   // <-- 본인 헤더 파일명으로 수정하세요

#include <cerrno>
#include <cstring>
#include <optional>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>


// -------------------------
// Daemon
// -------------------------

Daemon::~Daemon()
{
    // “감시” 기능은 없지만, 소멸 시 최소한 SIGTERM은 보내는 편이 안전합니다.
    // 원치 않으면 TerminateAll 호출을 지우셔도 됩니다.
    TerminateAll(15 /*SIGTERM*/);
}

std::vector<char*> Daemon::BuildArgv_(const std::string& exec, const std::vector<std::string>& args)
{
    std::vector<char*> argv;
    argv.reserve(1 + args.size() + 1);
    argv.push_back(const_cast<char*>(exec.c_str()));
    for (const auto& a : args) {
        argv.push_back(const_cast<char*>(a.c_str()));
    }
    argv.push_back(nullptr);
    return argv;
}

std::optional<int> Daemon::Spawn(const ProcessSpec& spec)
{
    m_errno = 0;

    if (spec.name.empty() || spec.exec_path.empty()) {
        m_errno = EINVAL;
        return std::nullopt;
    }
    if (m_procs.find(spec.name) != m_procs.end()) {
        m_errno = EEXIST;
        return std::nullopt;
    }

    // fork 전에 spec을 저장해두면 부모 상태 관리가 단순해집니다.
    m_specs[spec.name] = spec;

    pid_t pid = ::fork();
    if (pid < 0) {
        m_errno = errno;
        m_specs.erase(spec.name);
        return std::nullopt;
    }

    if (pid == 0) {
        if (spec.create_process_group) ::setpgid(0, 0);

        for (const auto& [k, v] : spec.env) ::setenv(k.c_str(), v.c_str(), 1);
        ::setenv("WORKER_NAME", spec.name.c_str(), 1);

        std::string exec = spec.exec_path;
        std::vector<std::string> args = spec.args;

        if (spec.launch_in_terminal) {
            const std::string cmd = BuildCommandLineForBash(spec);
            const std::string tail = "; exec bash";
            exec = "/usr/bin/gnome-terminal";
            args = {"--", "bash", "-lc", cmd + tail};
        }

        auto argv = BuildArgv_(exec, args);
        extern char** environ;
        ::execve(exec.c_str(), argv.data(), environ);
        _exit(127);
    }

    // ----- parent -----
    ProcessInfo info;
    info.pid = static_cast<int>(pid);
    info.running = true;
    info.exited = false;
    info.exit_code = 0;
    info.term_signal = 0;

    m_procs[spec.name] = info;
    return info.pid;
}

bool Daemon::Signal(const std::string& name, int sig)
{
    m_errno = 0;

    auto it = m_procs.find(name);
    if (it == m_procs.end()) {
        m_errno = ENOENT;
        return false;
    }

    const int pid = it->second.pid;
    if (pid <= 0) {
        m_errno = ESRCH;
        return false;
    }

    if (::kill(pid, sig) != 0) {
        m_errno = errno;
        return false;
    }
    return true;
}

std::vector<std::string> Daemon::Reap()
{
    m_errno = 0;
    std::vector<std::string> changed;

    int status = 0;
    while (true) {
        pid_t pid = ::waitpid(-1, &status, WNOHANG);
        if (pid == 0) break;               // no more exited children
        if (pid < 0) {
            if (errno == EINTR) continue;
            if (errno == ECHILD) break;    // no children
            m_errno = errno;
            break;
        }

        // pid로 어떤 프로세스인지 찾음
        for (auto& [name, info] : m_procs) {
            if (info.pid == static_cast<int>(pid)) {
                info.running = false;
                info.exited = true;
                info.exit_code = 0;
                info.term_signal = 0;

                if (WIFEXITED(status)) {
                    info.exit_code = WEXITSTATUS(status);
                } else if (WIFSIGNALED(status)) {
                    info.term_signal = WTERMSIG(status);
                }
                changed.push_back(name);
                break;
            }
        }
    }

    return changed;
}

bool Daemon::RefreshRunning(const std::string& name)
{
    m_errno = 0;

    auto it = m_procs.find(name);
    if (it == m_procs.end()) {
        m_errno = ENOENT;
        return false;
    }

    const int pid = it->second.pid;
    if (pid <= 0) {
        it->second.running = false;
        return false;
    }

    if (::kill(pid, 0) == 0) {
        it->second.running = true;
        return true;
    }

    if (errno == EPERM) {
        // 권한은 없지만 프로세스는 존재
        it->second.running = true;
        return true;
    }

    it->second.running = false;
    m_errno = errno;
    return false;
}

std::optional<ProcessInfo> Daemon::Get(const std::string& name) const
{
    auto it = m_procs.find(name);
    if (it == m_procs.end()) return std::nullopt;
    return it->second;
}

void Daemon::TerminateAll(int sig)
{
    // 실패해도 전체 종료는 계속 진행
    for (auto& [name, info] : m_procs) {
        (void)name;
        if (info.pid > 0) {
            ::kill(info.pid, sig);
        }
    }
}
