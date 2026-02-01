#pragma once

#include "lcm_subscriber/gps_lcm_type/gps_t.hpp"

#include <functional>
#include <optional>
#include <string>

#include <lcm/lcm-cpp.hpp>      // LCM C++ wrapper
#include <yaml-cpp/yaml.h>      // yaml-cpp

struct LcmSubscriberConfig
{
    std::string lcm_url;    // e.g. "udpm://239.255.76.67:7667?ttl=1"
    std::string channel;    // e.g. "GPS"
};

std::optional<LcmSubscriberConfig> LoadLcmSubscriberConfig(const std::string& yaml_path);

class LcmSubscriber
{
public:
    using Callback = std::function<void(const void* data, std::size_t size, const std::string& channel)>;

    LcmSubscriber() = default;
    ~LcmSubscriber() = default;

    bool InitFromYaml(const std::string& yaml_path);

    bool Init(const LcmSubscriberConfig& cfg);

    void SetCallback(Callback cb) { cb_ = std::move(cb); }

    int HandleOnce(int timeout_ms = -1);
    
    const LcmSubscriberConfig& config() const { return cfg_; }

private:
    void OnMsg_(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const gps_lcm_type::gps_t* msg);
private:
    LcmSubscriberConfig cfg_{};
    Callback cb_{};

    std::unique_ptr<lcm::LCM> lcm_{};
    lcm::Subscription* sub_{nullptr};
};
