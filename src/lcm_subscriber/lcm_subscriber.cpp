#include "lcm_subscriber/lcm_subscriber.hpp"

#include <utility>

#include <yaml-cpp/yaml.h>
#include <lcm/lcm-cpp.hpp>
#include <iostream>

// =========================
// YAML config loader
// =========================
std::optional<LcmSubscriberConfig> LoadLcmSubscriberConfig(const std::string& yaml_path)
{
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        std::cout << root << std::endl;
        LcmSubscriberConfig cfg;

        // lcm.url
        if (!root["lcm"] || !root["lcm"]["url"]) {
            return std::nullopt;
        }
        cfg.lcm_url = root["lcm"]["url"].as<std::string>();

        // subscriber.channel
        if (!root["subscriber"] || !root["subscriber"]["channel"]) {
            return std::nullopt;
        }
        cfg.channel = root["subscriber"]["channel"].as<std::string>();


        return cfg;
    } catch (const YAML::Exception&) {
        return std::nullopt;
    }
}

// =========================
// LcmSubscriber
// =========================
bool LcmSubscriber::InitFromYaml(const std::string& yaml_path)
{
    auto cfg = LoadLcmSubscriberConfig(yaml_path);
    if (!cfg) return false;
    return Init(*cfg);
}

bool LcmSubscriber::Init(const LcmSubscriberConfig& cfg)
{
    cfg_ = cfg;

    lcm_ = std::make_unique<lcm::LCM>(cfg_.lcm_url);
    if (!lcm_ || !lcm_->good()) {
        lcm_.reset();
        return false;
    }

    // Raw subscription callback signature:
    //   void handler(const lcm::ReceiveBuffer* rbuf, const std::string& chan)
    sub_ = lcm_->subscribe(cfg_.channel, &LcmSubscriber::OnMsg_, this);
    if (!sub_) {
        lcm_.reset();
        std::cerr << "[ERROR] subscription has not been made!\n";
        return false;
    }
    std::cout << "[INFO] subscription has been made!\n";
    return true;
}

int LcmSubscriber::HandleOnce(int timeout_ms)
{
    if (!lcm_) return -1;

    if (timeout_ms < 0) {
        // blocking
        return lcm_->handle();
    }
    return lcm_->handleTimeout(timeout_ms);
}


void LcmSubscriber::OnMsg_(const lcm::ReceiveBuffer* rbuf,
                const std::string& channel, 
                const gps_lcm_type::gps_t* msg)
{
    if (!rbuf) 
    {
        std::cerr << "[ERROR] lcm handle error\n";
        return;
    }
    // rbuf->data : raw bytes (void*)
    // rbuf->data_size : length
    std::cout << "[INFO] lcm handle\n";
    if (cb_) {
        cb_(rbuf->data, static_cast<std::size_t>(rbuf->data_size), channel);
    }
}
