#include "lcm_subscriber/lcm_subscriber.hpp"

#include "shm_utils.hpp"
#include "shared_memory.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <string>
#include <chrono>
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

// ANSI escape helpers
static void ClearScreenAndHome()
{
    // Clear screen + move cursor to home
    std::cout << "\033[2J\033[H";
}

static void HideCursor(bool hide)
{
    if (hide) std::cout << "\033[?25l";
    else      std::cout << "\033[?25h";
}

// Shared latest message snapshot
struct LatestGps
{
    bool has_msg{false};
    std::chrono::steady_clock::time_point last_rx_tp{};
    gps_lcm_type::gps_t msg{};
};

static void PrintDashboard(const LatestGps& s, const std::string& channel, std::size_t last_size_bytes)
{
    ClearScreenAndHome();

    // header
    std::cout << "==== GPS LCM Monitor (no-scroll dashboard) ====\n";
    std::cout << "Channel      : " << channel << "\n";
    std::cout << "Last size    : " << last_size_bytes << " bytes\n";

    if (!s.has_msg) {
        std::cout << "Status       : waiting for first message...\n";
        std::cout << "==============================================\n";
        std::cout.flush();
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - s.last_rx_tp).count();

    const auto& m = s.msg;

    std::cout << "Status       : OK\n";
    std::cout << "Age          : " << age_ms << " ms since last RX\n";
    std::cout << "----------------------------------------------\n";

    std::cout << std::fixed << std::setprecision(3);

    // Epoch/time
    std::cout << "epoch        : " << m.epoch << "\n";
    std::cout << "timestamp(ns): " << m.timestamp << "\n";

    // UCM
    std::cout << "is_ucm_valid : " << (m.is_ucm_valid ? "true" : "false") << "\n";
    std::cout << "ucm_x [m]    : " << m.ucm_x << "\n";
    std::cout << "ucm_y [m]    : " << m.ucm_y << "\n";

    std::cout << "----------------------------------------------\n";

    // GGA
    std::cout << "is_gga_alive : " << (m.is_gga_alive ? "true" : "false") << "\n";
    std::cout << std::setprecision(8);
    std::cout << "gga_lat [deg]: " << m.gga_lat << "\n";
    std::cout << "gga_lon [deg]: " << m.gga_lon << "\n";
    std::cout << std::setprecision(3);
    std::cout << "gga_fixq     : " << m.gga_fixq << "\n";
    std::cout << "gga_nsat     : " << m.gga_nsat << "\n";
    std::cout << "gga_hdop     : " << m.gga_hdop << "\n";
    std::cout << "gga_alt [m]  : " << m.gga_alt << "\n";
    std::cout << "gga_geoid [m]: " << m.gga_geoid << "\n";

    std::cout << "----------------------------------------------\n";

    // RMC
    std::cout << "is_rmc_alive : " << (m.is_rmc_alive ? "true" : "false") << "\n";
    std::cout << std::setprecision(3);
    std::cout << "sog [m/s]    : " << m.sog_mps << "\n";
    std::cout << "cog [deg]    : " << m.cog_deg << "\n";

    std::cout << "==============================================\n";
    std::cout.flush();
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config.yaml>\n";
        return 1;
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

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

    // shared state for dashboard
    std::mutex mtx;
    LatestGps latest;
    std::size_t last_size_bytes = 0;

    // LCM callback: just store latest + write SHM
    subscriber.SetCallback(
        [&mtx, &latest, &last_size_bytes, shm]
        (const void* /*data*/, std::size_t size, const std::string& /*channel*/, const gps_lcm_type::gps_t* msg)
        {
            {
                std::lock_guard<std::mutex> lk(mtx);
                latest.has_msg = true;
                latest.last_rx_tp = std::chrono::steady_clock::now();
                latest.msg = *msg;  // copy snapshot
                last_size_bytes = size;
            }

            // SHM write (필요한 필드만)
            shm->gps_data.ucm_x = msg->ucm_x;
            shm->gps_data.ucm_y = msg->ucm_y;
            shm->gps_data.timestamp = msg->timestamp;
        }
    );

    // terminal cosmetics
    HideCursor(true);

    // -------------------------
    // 3) Main loop
    // -------------------------
    const int handle_timeout_ms = 20;   // lcm handle timeout
    const int ui_period_ms      = 50;   // UI refresh period (20 Hz)

    auto next_ui = std::chrono::steady_clock::now();

    while (g_run.load(std::memory_order_relaxed)) {
        int ret = subscriber.HandleOnce(handle_timeout_ms);
        if (ret < 0) {
            // 에러는 화면에 계속 찍으면 흐려지므로, dashboard에서 age로도 알 수 있게 하고 여기선 종료
            break;
        }

        auto now = std::chrono::steady_clock::now();
        if (now >= next_ui) {
            LatestGps snap;
            std::size_t snap_size = 0;
            {
                std::lock_guard<std::mutex> lk(mtx);
                snap = latest;
                snap_size = last_size_bytes;
            }
            PrintDashboard(snap, subscriber.config().channel, snap_size);
            next_ui = now + std::chrono::milliseconds(ui_period_ms);
        }
    }

    // -------------------------
    // 4) Shutdown
    // -------------------------
    HideCursor(false);
    std::cout << "\n[INFO] LCM subscriber exiting\n";
    shm_utils::CloseShm(shm);
    return 0;
}
