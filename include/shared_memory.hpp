#pragma once

#include <string>
#include <cstdint>

#pragma once
using float32_t = float;
using float64_t = double;


struct GpsData
{
    
    double   ucm_x;
    double   ucm_y; 
    int64_t  timestamp{0};
};

struct ShmData
{
    char name[32] = {0};  
    GpsData gps_data;

    std::uint64_t counter{0};
};
