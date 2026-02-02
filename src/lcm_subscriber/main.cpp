#include "lcm_subscriber/lcm_subscriber.hpp"

#include "shm_utils.hpp"
#include "shared_memory.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

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
        std::cerr << "Usage: " << argv[0] << " <config.yaml>\n";
        return 1;
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    std::cout << "[LCM subscriber] LCM subscriber Start\n";

    const std::string yaml_path = argv[1];

    // -------------------------
    // 1) SHM attach (Open-only)
    // -------------------------
    const char* shm_name = GetShmName();
    ShmData* shm = shm_utils::OpenShm(shm_name);
    if (!shm) {
        std::cerr << "[ERROR] Failed to open SHM: " << shm_name
                  << " (is daemon running and created it?)\n";
        return 1;
    }
    std::cout << "[INFO] SHM opened: " << shm_name << "\n";

    // -------------------------
    // 2) LCM subscriber init
    // -------------------------
    LcmSubscriber subscriber;
    if (!subscriber.InitFromYaml(yaml_path)) {
        std::cerr << "[ERROR] Failed to initialize LcmSubscriber from "
                  << yaml_path << "\n";
        shm_utils::CloseShm(shm);
        return 1;
    }

    // 메시지 수신 콜백 등록
    subscriber.SetCallback(
        [shm](const void* data, std::size_t size, const std::string& channel, const gps_lcm_type::gps_t* msg)
        {
            std::cout << "[LCM] channel=" << channel
                      << " size=" << size << " bytes\n"
                      << " ucm_x=" << msg->ucm_x << "[m]\n"
                      << " ucm_y=" << msg->ucm_y << "[m]\n"
                      << " timestemp=" << msg->timestamp << "\n";
            shm->gps_data.ucm_x = msg->ucm_x;
            shm->gps_data.ucm_y = msg->ucm_y;
            shm->gps_data.timestamp = msg->timestamp;
            (void)data;
            (void)size;
        }
    );

    std::cout << "[INFO] LCM subscriber started. Channel: "
              << subscriber.config().channel << "\n";

    // -------------------------
    // 3) Main loop
    // -------------------------
    while (g_run.load(std::memory_order_relaxed)) {
        int ret = subscriber.HandleOnce(100);  // 100 ms
        if (ret < 0) {
            std::cerr << "[ERROR] lcm handle error\n";
            break;
        }
    }

    // -------------------------
    // 4) Shutdown
    // -------------------------
    std::cout << "[INFO] LCM subscriber exiting\n";
    shm_utils::CloseShm(shm);
    return 0;
}
