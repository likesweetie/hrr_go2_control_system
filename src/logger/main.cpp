#include "logger/logger.hpp"

#include "shm_utils.hpp"
#include "shared_memory.hpp"

#include <atomic>
#include <csignal>
#include <chrono>
#include <iostream>
#include <thread>

static std::atomic<bool> g_run{true};

static void HandleSignal(int)
{
    g_run.store(false, std::memory_order_relaxed);
}

static const char* GetShmName()
{
    return "/shm0";
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <logger_config.yaml>\n";
        return 1;
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    const std::string yaml_path = argv[1];

    // -------------------------
    // 1) YAML load
    // -------------------------
    auto cfg_opt = LoadCsvLoggerConfig(yaml_path);
    if (!cfg_opt) {
        std::cerr << "[ERROR] Failed to load logger config from: " << yaml_path << "\n";
        return 1;
    }
    const CsvLoggerConfig cfg = *cfg_opt;

    // -------------------------
    // 2) SHM open
    // -------------------------
    const char* shm_name = GetShmName();
    ShmData* shm = shm_utils::OpenShm(shm_name);
    if (!shm) {
        std::cerr << "[ERROR] Failed to open SHM: " << shm_name
                  << " (is daemon running and created it?)\n";
        return 1;
    }

    // -------------------------
    // 3) Logger open
    // -------------------------
    CsvLogger logger;
    if (!logger.Open(cfg)) {
        shm_utils::CloseShm(shm);
        return 1;
    }

    std::cout << "[INFO] Logging started\n"
              << "  period_sec = " << cfg.period_sec << "\n"
              << "  output     = " << logger.file_path() << "\n"
              << "  fsync      = " << (cfg.fsync_each_row ? "true" : "false") << "\n";

    // -------------------------
    // 4) Periodic loop
    // -------------------------
    const auto period = std::chrono::duration<double>(cfg.period_sec);
    auto next_tp = std::chrono::steady_clock::now();

    while (g_run.load(std::memory_order_relaxed)) {
        next_tp += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

        logger.LogOnce(shm);

        // drift 최소화: sleep_until
        std::this_thread::sleep_until(next_tp);
    }

    // -------------------------
    // 5) Shutdown
    // -------------------------
    std::cout << "\n[INFO] Logger exiting...\n";
    logger.Close();
    shm_utils::CloseShm(shm);
    return 0;
}
