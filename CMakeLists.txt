# project(low_level)
# cmake_minimum_required(VERSION 3.5)

# SET(CMAKE_CXX_STANDARD 17)

# set(CMAKE_BUILD_TYPE Release)

# set(CMAKE_PREFIX_PATH /home/unitree/.local/lib/python3.8/site-packages/torch)
# set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

# # find_package(CUDA REQUIRED)
# find_package(Torch REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(Python REQUIRED COMPONENTS Interpreter Development)
# find_package(PkgConfig REQUIRED)
# find_package(lcm REQUIRED)

# include_directories(${EIGEN3_INCLUDE_DIR})

# include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
# link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

# include_directories(
#     ${Python_INCLUDE_DIRS}
#     ${EIGEN3_INCLUDE_DIR}
#     include
#     include/Tools
#     lcm_types
# )

# include_directories(
#     /home/unitree/.local/lib/python3.8/site-packages/torch/include
#     /home/unitree/.local/lib/python3.8/site-packages/torch/include/torch/csrc/api/include
# )

# add_subdirectory(include/Tools)

# add_library(low_level_lib
#   src/low_level/low_level.cpp
# )

# include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
# link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

# add_executable(low_level src/low_level/low_level.cpp)

# add_executable(main src/low_level/main.cpp)  

# target_link_libraries(low_level 
#   ${Python_LIBRARIES}
#   Tools
#   "${TORCH_LIBRARIES}"
#   lcm
# )

# target_link_libraries(main 
#   low_level_lib
#   ${Python_LIBRARIES}
#   Tools
#   "${TORCH_LIBRARIES}"
#   lcm
# )


project(low_level)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Release)

# find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lcm REQUIRED)
find_package(yaml-cpp REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    include
    include/Tools
    lcm_types
)
#---------------daemon------------------

add_executable(daemon
    src/daemon/main.cpp
    src/daemon/daemon.cpp
)
target_include_directories(daemon
    PRIVATE ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(daemon
    PRIVATE yaml-cpp::yaml-cpp
)
#---------------------------------
#-----------------lcm_subscriber----------------
add_executable(lcm_subscriber
    src/lcm_subscriber/main.cpp
    src/lcm_subscriber/lcm_subscriber.cpp
)
target_include_directories(lcm_subscriber
    PRIVATE ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(lcm_subscriber
    PRIVATE yaml-cpp::yaml-cpp lcm
)


#---------------------------------

# add_executable(low_level 
#               src/low_level/low_level.cpp
#               )


# add_executable(low_level src/low_level/low_level.cpp)
# add_executable(main src/low_level/main.cpp)  


# target_link_libraries(daemon 
  # low_level_lib
  # ${Python_LIBRARIES}
  # Tools
  # "${TORCH_LIBRARIES}"
  # lcm
# )