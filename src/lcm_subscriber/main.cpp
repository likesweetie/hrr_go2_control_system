#include "lcm_subscriber/lcm_subscriber.hpp"

#include <cstdlib>
#include <cstring>
#include <iostream>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config.yaml>\n";
        return 1;
    }
    std::cout << "[LCM subscriber] LCM subscriber Start\n";

    const std::string yaml_path = argv[1];

    LcmSubscriber subscriber;

    if (!subscriber.InitFromYaml(yaml_path)) {
        std::cerr << "[ERROR] Failed to initialize LcmSubscriber from "
                  << yaml_path << "\n";
        return 1;
    }

    // 메시지 수신 콜백 등록
    subscriber.SetCallback(
        [](const void* data, std::size_t size, const std::string& channel)
        {
            // 최소 동작 예시: payload 크기만 출력
            std::cout << "[LCM] channel=" << channel
                      << " size=" << size << " bytes\n";

            // 실제 사용 시:
            //  - 여기서 lcm-gen 메시지 디코딩
            //  - shm에 write
            //  - 제어/상태 업데이트
        }
    );

    std::cout << "[INFO] LCM subscriber started. Channel: "
              << subscriber.config().channel << "\n";

    // 메인 루프
    // - handleTimeout(): timeout 동안 메시지 기다림
    // - 반환값:
    //   > 0 : 메시지 처리됨
    //   = 0 : timeout
    //   < 0 : 에러
    while (true) {
        int ret = subscriber.HandleOnce(100);  // 100 ms

        if (ret < 0) {
            std::cerr << "[ERROR] lcm handle error\n";
            break;
        }
        // 다른 주기 작업이 필요하면 여기서 수행
        // 예: 상태 출력, watchdog kick 등
    }

    std::cout << "[INFO] LCM subscriber exiting\n";
    return 0;
}
