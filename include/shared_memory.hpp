#pragma once

#include <string>

struct GpsData
{
    double lat_deg{0.0};
    double lon_deg{0.0};
    double alt_m{0.0};

    float  vel_n_mps{0.0f};
    float  vel_e_mps{0.0f};
    float  vel_d_mps{0.0f};

    float  yaw_rad{0.0f};
    float  yaw_std_rad{0.0f};

    std::uint64_t timestamp_ns{0};
};

struct ShmData
{
    char name[32] = {0};  
    GpsData gps_data;

    std::uint64_t counter{0};
};
