#include "daemon/daemon.hpp"

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <unistd.h>

// -------------------------
// Globals
// -------------------------
static std::atomic<bool> g_run{true};
static std::atomic<int>  g_got_signal{0};
static Daemon d;

// async-signal-safe log
static void SafeWrite(const char* msg)
{
    ::write(STDERR_FILENO, msg, std::strlen(msg));
}

static void HandleSignal(int sig)
{
    g_got_signal.store(sig, std::memory_order_relaxed);
    g_run.store(false, std::memory_order_relaxed);
    SafeWrite("[WARN] signal caught, shutting down...\n");
}

// -------------------------
// YAML helpers
// -------------------------
static bool LoadShmConfig(const YAML::Node& cfg, ShmConfig& shm)
{
    if (!cfg["shm"]) return false;
    const auto& s = cfg["shm"];

    if (!s["name"] || !s["size_bytes"]) return false;

    shm.name = s["name"].as<std::string>();
    shm.size_bytes = s["size_bytes"].as<std::size_t>();

    if (s["create_if_missing"]) shm.create_if_missing = s["create_if_missing"].as<bool>();
    if (s["unlink_on_destroy"]) shm.unlink_on_destroy = s["unlink_on_destroy"].as<bool>();
    return true;
}

static bool LoadProcessSpecs(const YAML::Node& cfg, std::vector<ProcessSpec>& procs)
{
    if (!cfg["processes"] || !cfg["processes"].IsSequence()) return false;

    for (const auto& p : cfg["processes"]) {
        if (!p["name"] || (!p["exec_path"] && !p["exec"])) {
            std::cerr << "[ERROR] each process needs 'name' and 'exec_path'(or legacy 'exec')\n";
            return false;
        }

        ProcessSpec spec;
        spec.name = p["name"].as<std::string>();

        if (p["exec_path"]) spec.exec_path = p["exec_path"].as<std::string>();
        else               spec.exec_path = p["exec"].as<std::string>();

        if (p["args"]) {
            if (!p["args"].IsSequence()) {
                std::cerr << "[ERROR] args must be sequence\n";
                return false;
            }
            for (const auto& a : p["args"]) spec.args.push_back(a.as<std::string>());
        }


        if (p["launch_in_terminal"]) spec.launch_in_terminal = p["launch_in_terminal"].as<bool>();
        else                        spec.launch_in_terminal = false;


        if (p["env"]) {
            if (!p["env"].IsMap()) {
                std::cerr << "[ERROR] env must be map\n";
                return false;
            }
            for (auto it = p["env"].begin(); it != p["env"].end(); ++it) {
                spec.env[it->first.as<std::string>()] = it->second.as<std::string>();
            }
        }

        procs.push_back(std::move(spec));
    }
    return true;
}

// -------------------------
// Shutdown helper
// -------------------------
static void ShutdownAll(const ShmConfig& shm_cfg)
{
    // 1) graceful
    d.TerminateAll(SIGTERM);

    // 2) give some time
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 3) force
    d.TerminateAll(SIGKILL);

    // 4) shm close/unlink
    if (d.shm) shm_utils::CloseShm(d.shm);
    if (shm_cfg.unlink_on_destroy) shm_utils::DeleteShmByName(shm_cfg.name.c_str());
}

int main(int argc, char** argv)
{
    (void)argc; (void)argv;

    const std::string config_path = "../configs/daemon_config.yaml";

    // signals
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    // load YAML
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(config_path);
    } catch (const YAML::Exception& e) {
        std::cerr << "[ERROR] failed to load config: " << e.what() << "\n";
        return 1;
    }

    // shm
    ShmConfig shm_cfg;
    if (!LoadShmConfig(cfg, shm_cfg)) {
        std::cerr << "[ERROR] invalid shm config\n";
        return 1;
    }

    bool created = false;
    d.shm = shm_utils::CreateShm(shm_cfg.name.c_str(), shm_cfg.create_if_missing, &created);
    if (!d.shm) {
        std::cerr << "[ERROR] failed to create/open shm: " << shm_cfg.name << "\n";
        return 1;
    }

    if (created) {
        std::cout << "[INFO] SHM created: " << shm_cfg.name << "\n";
        std::memset(d.shm, 0, sizeof(ShmData));
        std::strncpy(d.shm->name, shm_cfg.name.c_str(), sizeof(d.shm->name) - 1);
        d.shm->name[sizeof(d.shm->name) - 1] = '\0';
    }

    // processes
    std::vector<ProcessSpec> procs;
    if (!LoadProcessSpecs(cfg, procs)) {
        std::cerr << "[ERROR] invalid process list\n";
        ShutdownAll(shm_cfg);
        return 1;
    }

    for (const auto& p : procs) {
        auto pid = d.Spawn(p);
        if (!pid) {
            std::cerr << "[WARN] spawn failed: " << p.name
                      << " errno=" << d.LastErrno() << " (" << std::strerror(d.LastErrno()) << ")\n";
        } else {
            std::cout << "[INFO] spawned: " << p.name << " pid=" << *pid << "\n";
        }
    }

    // main loop
    while (g_run.load(std::memory_order_relaxed)) {

        std::cout << d.shm->gps_data.timestamp <<std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const int sig = g_got_signal.load(std::memory_order_relaxed);
    std::cout << "[INFO] shutdown signal=" << sig << "\n";

    ShutdownAll(shm_cfg);
    return 0;
}
