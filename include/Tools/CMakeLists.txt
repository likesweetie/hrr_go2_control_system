# CMake 최소 버전 설정
cmake_minimum_required(VERSION 3.5)

# 라이브러리 이름 설정
add_library(Tools Deploy.cpp Deploy_High.cpp)

# PyTorch 경로 설정
include_directories(
    ${EIGEN3_INCLUDE_DIR}
)

# PyTorch 라이브러리 링크
target_link_libraries(Tools "${TORCH_LIBRARIES}")
