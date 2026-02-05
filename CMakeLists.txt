project(low_level)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_C_COMPILE_LAUNCHER ccache)
set(CMAKE_CXX_COMPILE_LAUNCHER ccache)

# set(CMAKE_PREFIX_PATH /home/unitree/.local/lib/python3.8/site-packages/torch)
# set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(lcm REQUIRED)

message(STATUS "Python_EXECUTABLE=${Python_EXECUTABLE}")

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
# find_package(Boost REQUIRED COMPONENTS program_options)

execute_process(
  COMMAND ${Python_EXECUTABLE} -c
          "import os, torch; print(os.path.dirname(torch.__file__))"
  OUTPUT_VARIABLE TORCH_PY_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# TorchConfig.cmake 탐색 힌트로 추가
list(PREPEND CMAKE_PREFIX_PATH "${TORCH_PY_DIR}")
find_package(Torch REQUIRED)
message(STATUS "Torch found. TORCH_PY_DIR=${TORCH_PY_DIR}")

add_subdirectory(include/Tools)

#---------------daemon------------------
add_executable(daemon
    src/daemon/main.cpp
    src/daemon/daemon.cpp
)
target_include_directories(daemon
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(daemon
    PRIVATE yaml-cpp rt
)
#---------------------------------


#-----------------lcm_subscriber----------------
add_executable(lcm_subscriber
    src/lcm_subscriber/main.cpp
    src/lcm_subscriber/lcm_subscriber.cpp
)
target_include_directories(lcm_subscriber
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(lcm_subscriber
    PRIVATE yaml-cpp lcm rt
)
#---------------------------------


#-----------------Logger----------------
add_executable(logger
    src/logger/main.cpp
)
target_include_directories(logger
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(logger
    PRIVATE yaml-cpp lcm rt
)
#---------------------------------

# -------- common include dirs (replacing global include_directories) ----------
set(LOW_LEVEL_COMMON_INCLUDES
    ${EIGEN3_INCLUDE_DIR}
    ${Python_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/Tools
    # ${CMAKE_SOURCE_DIR}/include/daemon
    # ${CMAKE_SOURCE_DIR}/include/lcm_subscriber
    ${CMAKE_SOURCE_DIR}/lcm_types
)

set(LOW_LEVEL_COMMON_LIBS
    torch
    unitree_sdk2
    ddsc
    ddscxx
    rt
    pthread
    yaml-cpp
)


# ---------------------------------------------------------------------------

add_library(low_level_lib
  src/low_level/low_level.cpp
)
target_include_directories(low_level_lib
    PRIVATE ${LOW_LEVEL_COMMON_INCLUDES}
)
target_link_libraries(low_level_lib
    PRIVATE ${LOW_LEVEL_COMMON_LIBS}
)

# target_precompile_headers(low_level_lib 
#                             PRIVATE 
#                             <torch/torch.h> 
#                             <Eigen/Dense>
#                             <unitree/robot/channel/channel_publisher.hpp>
#                             <unitree/robot/channel/channel_subscriber.hpp>
#                             <unitree/idl/go2/LowState_.hpp>
#                             <unitree/idl/go2/LowCmd_.hpp>
#                             <unitree/common/time/time_tool.hpp>
#                             <unitree/common/thread/thread.hpp>
#                             )

# add_executable(low_level src/low_level/low_level.cpp)
# target_include_directories(low_level
#     PRIVATE ${LOW_LEVEL_COMMON_INCLUDES}
# )
# target_link_libraries(low_level
#   PRIVATE
#     ${Python_LIBRARIES}
#     Tools
#     "${TORCH_LIBRARIES}"
#     lcm
#     ${LOW_LEVEL_COMMON_LIBS}
# )

add_executable(main src/low_level/main.cpp)
target_include_directories(main
    PRIVATE ${LOW_LEVEL_COMMON_INCLUDES}
)
target_link_libraries(main
  PRIVATE
    low_level_lib
    ${Python_LIBRARIES}
    Tools
    "${TORCH_LIBRARIES}"
    lcm
    ${LOW_LEVEL_COMMON_LIBS}
)

