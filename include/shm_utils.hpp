#pragma once

#include "shared_memory.hpp"

#include <cstddef>
#include <cerrno>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace shm_utils
{


inline ShmData* OpenShm(const char* name)
{
    int fd = shm_open(name, O_RDWR, 0666);
    if (fd < 0) return nullptr;

    void* addr = mmap(nullptr, sizeof(ShmData),
                      PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (addr == MAP_FAILED) return nullptr;

    auto* shm = reinterpret_cast<ShmData*>(addr);
    return shm;
}


inline ShmData* CreateShm(const char* name, bool create, bool* out_created)
{
    if (out_created) *out_created = false;

    int fd = -1;

    if (create) {
        // 1) "새로 생성"을 먼저 시도해서 out_created를 정확히 채움
        fd = shm_open(name, O_RDWR | O_CREAT | O_EXCL, 0666);
        if (fd >= 0) {
            if (out_created) *out_created = true;
        } else {
            // 이미 존재하면(exist) 그냥 open으로 폴백
            if (errno != EEXIST) {
                return nullptr;
            }
            fd = shm_open(name, O_RDWR, 0666);
            if (fd < 0) return nullptr;
        }
    } else {
        // 반드시 기존에 있어야 함
        fd = shm_open(name, O_RDWR, 0666);
        if (fd < 0) return nullptr;
    }

    // 2) 크기를 ShmData에 맞춤 (creator 역할이면 특히 필수)
    //    이미 존재하는 shm에 대해 ftruncate를 호출하는 것은 크기를 "설정"하는 동작입니다.
    //    (축소하면 데이터가 잘릴 수 있으니, 운영 정책에 따라 검증 후 실패로 바꿀 수도 있습니다.)
    if (ftruncate(fd, static_cast<off_t>(sizeof(ShmData))) != 0) {
        close(fd);
        return nullptr;
    }

    // 3) mmap으로 매핑
    void* addr = mmap(nullptr, sizeof(ShmData),
                      PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    close(fd);

    if (addr == MAP_FAILED) {
        return nullptr;
    }

    return static_cast<ShmData*>(addr);
}

inline void CloseShm(ShmData* shm)
{
    if (!shm) return;
    ::munmap(static_cast<void*>(shm), sizeof(ShmData));
}

inline void DeleteShmByName(const char* name)
{
    if (!name) return;
    ::shm_unlink(name);
}

}
