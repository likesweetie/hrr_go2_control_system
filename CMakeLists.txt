project(low_level)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# set(CMAKE_PREFIX_PATH /home/unitree/.local/lib/python3.8/site-packages/torch)
# set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

# find_package(CUDA REQUIRED)
# find_package(Torch REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lcm REQUIRED)
find_package(yaml-cpp REQUIRED)

# add_subdirectory(include/Tools)

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


# -------- common include dirs (replacing global include_directories) ----------
set(LOW_LEVEL_COMMON_INCLUDES
    ${EIGEN3_INCLUDE_DIR}
    ${Python_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    # ${CMAKE_SOURCE_DIR}/include/Tools
    ${CMAKE_SOURCE_DIR}/include/daemon
    ${CMAKE_SOURCE_DIR}/include/lcm_subscriber
    ${CMAKE_SOURCE_DIR}/lcm_types

    # /usr/local/include/ddscxx
    # /usr/local/include/iceoryx/v2.0.2

    # /home/unitree/.local/lib/python3.8/site-packages/torch/include
    # /home/unitree/.local/lib/python3.8/site-packages/torch/include/torch/csrc/api/include
)

# set(LOW_LEVEL_COMMON_LIBS
#     unitree_sdk2
#     ddsc
#     ddscxx
#     rt
#     pthread
# )

# ---------------------------------------------------------------------------

# add_library(low_level_lib
#   src/low_level/low_level.cpp
# )
# target_include_directories(low_level_lib
#     PRIVATE ${LOW_LEVEL_COMMON_INCLUDES}
# )
# target_link_libraries(low_level_lib
#     PRIVATE ${LOW_LEVEL_COMMON_LIBS}
# )

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

# add_executable(main src/low_level/main.cpp)
# target_include_directories(main
#     PRIVATE ${LOW_LEVEL_COMMON_INCLUDES}
# )
# target_link_libraries(main
#   PRIVATE
#     low_level_lib
#     ${Python_LIBRARIES}
#     Tools
#     "${TORCH_LIBRARIES}"
#     lcm
#     ${LOW_LEVEL_COMMON_LIBS}
# )
