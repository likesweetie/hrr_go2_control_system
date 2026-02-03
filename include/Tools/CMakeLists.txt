# CMake 최소 버전 설정
cmake_minimum_required(VERSION 3.5)

# 라이브러리 이름 설정
add_library(Tools Deploy.cpp)
add_library(Tools Deploy_High.cpp)

# PyTorch 경로 설정
include_directories(
    /home/unitree/.local/lib/python3.8/site-packages/torch/include
    /home/unitree/.local/lib/python3.8/site-packages/torch/include/torch/csrc/api/include
)

# PyTorch 라이브러리 링크
target_link_libraries(Tools "${TORCH_LIBRARIES}")
