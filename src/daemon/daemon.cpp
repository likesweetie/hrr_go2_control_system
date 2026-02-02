#include "daemon/daemon.hpp"

#include <cerrno>
#include <cstring>
#include <optional>

#include <sys/wait.h>
#include <unistd.h>

// -------------------------
// Daemon
// -------------------------

Daemon::~Daemon()
{
    // 데몬 종료 시 등록된 프로세스들을 이름(comm) 기반으로 SIGTERM
    TerminateAll(15 /*SIGTERM*/);
}

std::vector<char*> Daemon::BuildArgv_(const std::string& exec, const std::vector<std::string>& args)
{
    std::vector<char*> argv;
    argv.reserve(1 + args.size() + 1);
    argv.push_back(const_cast<char*>(exec.c_str()));
    for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);
    return argv;
}

bool Daemon::PkillExact_(const std::string& comm_name, int sig)
{
    m_errno = 0;
    if (comm_name.empty()) {
        m_errno = EINVAL;
        return false;
    }

    pid_t child = ::fork();
    if (child < 0) {
        m_errno = errno;
        return false;
    }

    if (child == 0) {
        // pkill -<sig> -x <comm_name>
        std::string sig_opt = "-" + std::to_string(sig);
        ::execlp("pkill",
                 "pkill",
                 sig_opt.c_str(),
                 "-x",
                 comm_name.c_str(),
                 (char*)nullptr);
        _exit(127);
    }

    int status = 0;
    if (::waitpid(child, &status, 0) < 0) {
        m_errno = errno;
        return false;
    }

    // pkill exit code:
    // 0: match found and signaled
    // 1: no processes matched
    // 2: error
    if (WIFEXITED(status)) {
        const int ec = WEXITSTATUS(status);
        if (ec == 0) return true;
        if (ec == 1) return false;     // not found
        m_errno = ECHILD;              // pkill internal error (not a real errno)
        return false;
    }

    m_errno = EINTR;
    return false;
}

std::optional<int> Daemon::Spawn(const ProcessSpec& spec)
{
    m_errno = 0;

    if (spec.name.empty() || spec.exec_path.empty()) {
        m_errno = EINVAL;
        return std::nullopt;
    }

    // 이름 충돌 방지(논리 이름 = comm 이름 가정)
    if (m_specs.find(spec.name) != m_specs.end()) {
        m_errno = EEXIST;
        return std::nullopt;
    }

    // spec 저장(나중 TerminateAll에서 이름으로 kill)
    m_specs[spec.name] = spec;

    pid_t pid = ::fork();
    if (pid < 0) {
        m_errno = errno;
        m_specs.erase(spec.name);
        return std::nullopt;
    }

    if (pid == 0) {
        // ----- child -----
        for (const auto& [k, v] : spec.env) ::setenv(k.c_str(), v.c_str(), 1);
        ::setenv("WORKER_NAME", spec.name.c_str(), 1);

        // ★ 새 터미널에 띄우고 싶으면 gnome-terminal을 exec
        if (spec.launch_in_terminal) {
            // BuildCommandLineForBash(spec)는 daemon.hpp에 이미 정의되어 있는 helper 사용
            const std::string cmd = BuildCommandLineForBash(spec);

            const std::string term = "/usr/bin/gnome-terminal";
            std::vector<std::string> targs = {"--", "bash", "-lc", cmd};

            auto argv = BuildArgv_(term, targs);
            extern char** environ;
            ::execve(term.c_str(), argv.data(), environ);
            _exit(127);
        }

        // ★ 터미널 없이 그냥 직접 실행
        auto argv = BuildArgv_(spec.exec_path, spec.args);
        extern char** environ;
        ::execve(spec.exec_path.c_str(), argv.data(), environ);
        _exit(127);
    }

    // ----- parent -----
    return static_cast<int>(pid);
}

bool Daemon::Signal(const std::string& proc_name, int sig)
{
    return PkillExact_(proc_name, sig);
}

void Daemon::TerminateAll(int sig)
{
    // 등록된 프로세스들을 “이름(comm)”으로 종료
    for (const auto& [name, spec] : m_specs) {
        (void)spec;
        (void)PkillExact_(name, sig);
    }
}
