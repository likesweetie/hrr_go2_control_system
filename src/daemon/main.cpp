#include "daemon/daemon.hpp"

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <csignal>
#include <iostream>
#include <string>
#include <cstring>  
#include <thread>
#include <chrono>
#include <memory.h>

static std::atomic<bool> g_run{true};

static const char* shm_name = "/shm0";
Daemon d;

static void HandleSignal(int)
{
    g_run.store(false);
    std::cout << "[ERROR] Daemon Interrupted!\n";
    d.TerminateAll();   // SIGTERM
    shm_utils::CloseShm(d.shm);
    shm_utils::DeleteShmByName(shm_name);
    
}

// -------------------------
// YAML loader helpers
// -------------------------
static bool LoadShmConfig(const YAML::Node& cfg, ShmConfig& shm)
{
    if (!cfg["shm"]) return false;

    const auto& s = cfg["shm"];
    shm.name = s["name"].as<std::string>();
    shm.size_bytes = s["size_bytes"].as<std::size_t>();
    shm.create_if_missing = s["create_if_missing"].as<bool>();
    shm.unlink_on_destroy = s["unlink_on_destroy"].as<bool>();
    return true;
}

static bool LoadProcessSpecs(const YAML::Node& cfg,
                             std::vector<ProcessSpec>& procs)
{
    if (!cfg["processes"] || !cfg["processes"].IsSequence())
        return false;

    for (const auto& p : cfg["processes"]) {
        ProcessSpec spec;
        spec.name = p["name"].as<std::string>();
        spec.exec_path = p["exec"].as<std::string>();

        if (p["args"]) {
            for (const auto& a : p["args"]) {
                spec.args.push_back(a.as<std::string>());
            }
        }

        if (p["launch_in_terminal"]) 
        {
            spec.launch_in_terminal =true;
        }
        procs.push_back(std::move(spec));
    }
    return true;
}

int main()
{
    const std::string config_path = "../configs/daemon_config.yaml";

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(config_path);
    } catch (const YAML::Exception& e) {
        std::cerr << "[ERROR] Failed to load daemon config: "
                  << e.what() << "\n";
        return 1;
    }

    // -------------------------
    // 1) SHM
    // -------------------------

    ShmConfig shm_config;
    if (!LoadShmConfig(cfg, shm_config)) {
        std::cerr << "[ERROR] Invalid shm config\n";
        return 1;
    }

    bool created = false;
    d.shm = shm_utils::CreateShm(shm_name, true, &created);
    if (!d.shm) { /* error */ }
    if (created) {
        std::cout << "[INFO] Shared Memory Created\n";
        memset(d.shm, 0, sizeof(ShmData));
        std::strncpy(d.shm->name, "/shm0", sizeof(d.shm->name) - 1);
        d.shm->name[sizeof(d.shm->name) - 1] = '\0';
    }




    // -------------------------
    // 2) Processes
    // -------------------------
    std::vector<ProcessSpec> processes;
    if (!LoadProcessSpecs(cfg, processes)) {
        std::cerr << "[ERROR] Invalid process list\n";
        return 1;
    }

    for (const auto& p : processes) {
        if (!d.Spawn(p)) {
            std::cerr << "[WARN] Failed to spawn process: "
                      << p.name << "\n";
        } else {
            std::cout << "[INFO] Spawned process: "
                      << p.name << "\n";
        }
    }

    // -------------------------
    // 3) Main loop (reap only)
    // -------------------------
    while (g_run.load()) {
        d.Reap();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // -------------------------
    // 4) Shutdown
    // -------------------------
    std::cout << "[INFO] Daemon shutting down\n";
    d.TerminateAll();   // SIGTERM
    shm_utils::CloseShm(d.shm);
    shm_utils::DeleteShmByName(shm_name);
    return 0;
}
