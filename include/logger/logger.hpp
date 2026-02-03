#pragma once

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>

#include <fcntl.h>   // ::open
#include <unistd.h>  // ::close, ::fsync

#include "shared_memory.hpp"  // ShmData 정의가 여기 있다고 가정

// =========================
// Config
// =========================
struct CsvLoggerConfig
{
    std::string path;         // directory
    std::string filename;     // file name
    double period_sec{0.1};   // default 0.1s
    bool fsync_each_row{true};
};

// =========================
// YAML loader
// =========================
inline std::optional<CsvLoggerConfig> LoadCsvLoggerConfig(const std::string& yaml_path)
{
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        CsvLoggerConfig cfg;

        if (!root["logger"]) {
            std::cerr << "[ERROR] YAML: missing 'logger'\n";
            return std::nullopt;
        }
        auto n = root["logger"];

        if (!n["path"] || !n["filename"]) {
            std::cerr << "[ERROR] YAML: missing 'logger.path' or 'logger.filename'\n";
            return std::nullopt;
        }

        cfg.path = n["path"].as<std::string>();
        cfg.filename = n["filename"].as<std::string>();

        if (n["period_sec"]) cfg.period_sec = n["period_sec"].as<double>();
        if (n["fsync"])      cfg.fsync_each_row = n["fsync"].as<bool>();

        if (cfg.period_sec <= 0.0) {
            std::cerr << "[ERROR] YAML: 'logger.period_sec' must be > 0\n";
            return std::nullopt;
        }

        return cfg;
    } catch (const YAML::Exception& e) {
        std::cerr << "[ERROR] YAML parse error: " << e.what() << "\n";
        return std::nullopt;
    }
}

// =========================
// CsvLogger
// =========================
class CsvLogger
{
public:
    CsvLogger() = default;
    ~CsvLogger() { Close(); }

    CsvLogger(const CsvLogger&) = delete;
    CsvLogger& operator=(const CsvLogger&) = delete;

    bool Open(const CsvLoggerConfig& cfg)
    {
        Close();  // just in case

        cfg_ = cfg;

        namespace fs = std::filesystem;
        std::error_code ec;

        // mkdir -p
        fs::create_directories(cfg_.path, ec);
        if (ec) {
            std::cerr << "[ERROR] create_directories failed: " << cfg_.path
                      << " : " << ec.message() << "\n";
            return false;
        }

        // 1) 충돌 없는 파일명 결정
        file_path_ = MakeUniquePath_(fs::path(cfg_.path), cfg_.filename);

        // 2) 파일 오픈 (append)
        ofs_.open(file_path_, std::ios::out | std::ios::app);
        if (!ofs_.is_open()) {
            std::cerr << "[ERROR] failed to open file: " << file_path_ << "\n";
            return false;
        }

        // 3) (옵션) fsync용 fd를 별도로 open
        fd_ = -1;
        if (cfg_.fsync_each_row) {
            fd_ = ::open(file_path_.c_str(), O_WRONLY | O_APPEND);
            if (fd_ < 0) {
                std::cerr << "[WARN] fsync enabled but open() failed; will flush only.\n";
            }
        }

        // 4) 새 파일이므로 헤더 작성
        WriteHeader_();
        Flush_();

        is_open_ = true;
        std::cout << "[INFO] CSV logger opened: " << file_path_ << "\n";
        return true;
    }

    void Close()
    {
        if (ofs_.is_open()) {
            Flush_();
            ofs_.close();
        }
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        is_open_ = false;
    }

    bool IsOpen() const { return is_open_; }

    const std::string& file_path() const { return file_path_; }

    // 주기적으로 1 row 기록
    void LogOnce(const ShmData* shm)
    {
        if (!is_open_ || !shm) return;

        // 호스트 측 타임스탬프(로거가 기록한 시각)
        const auto now_sys = std::chrono::system_clock::now();
        const std::int64_t host_time_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now_sys.time_since_epoch()).count();

        // ===== 예시: SHM gps_data만 기록 =====
        // 실제 shared_memory.hpp 필드에 맞게 수정/확장하세요.
        const std::int64_t gps_ts = static_cast<std::int64_t>(shm->gps_data.timestamp);
        const double ucm_x = shm->gps_data.ucm_x;
        const double ucm_y = shm->gps_data.ucm_y;

        ofs_ << host_time_ns << ","
             << gps_ts << ","
             << ucm_x << ","
             << ucm_y
             << "\n";

        std::cout << host_time_ns << ","
             << gps_ts << ","
             << ucm_x << ","
             << ucm_y
             << "\n";

        Flush_();
    }

private:
    // filename 충돌 해결:
    // - 없으면 그대로 사용
    // - 있으면 stem_copy.ext, stem_copy2.ext, ...
    static std::string MakeUniquePath_(const std::filesystem::path& dir,
                                       const std::string& filename)
    {
        namespace fs = std::filesystem;

        fs::path base(filename);
        const std::string stem = base.stem().string();
        const std::string ext  = base.extension().string();  // ".csv" or ""

        fs::path candidate = dir / filename;
        std::error_code ec;

        if (!fs::exists(candidate, ec) || ec) {
            // exists 체크 실패(ec!=0)도 "없음 취급"으로 진행 (보수적으로는 실패 시 return candidate)
            return candidate.string();
        }

        int idx = 1;
        while (true) {
            std::ostringstream oss;
            if (idx == 1) oss << stem << "_copy" << ext;
            else          oss << stem << "_copy" << idx << ext;

            candidate = dir / oss.str();
            ec.clear();
            if (!fs::exists(candidate, ec) || ec) {
                return candidate.string();
            }
            ++idx;
        }
    }

    void WriteHeader_()
    {
        // 필요한 필드에 맞춰 확장
        ofs_ << "host_time_ns,gps_timestamp,ucm_x,ucm_y\n";
    }

    void Flush_()
    {
        // 1) 유저 공간 flush
        ofs_.flush();

        // 2) 커널 버퍼까지 강제 반영 (옵션)
        if (cfg_.fsync_each_row && fd_ >= 0) {
            ::fsync(fd_);
        }
    }

private:
    CsvLoggerConfig cfg_{};
    std::ofstream ofs_;
    std::string file_path_;
    bool is_open_{false};

    int fd_{-1};  // fsync용 별도 fd
};
