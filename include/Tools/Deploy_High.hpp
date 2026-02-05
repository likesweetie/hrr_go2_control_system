#ifndef HIGHDEPLOY_H
#define HIGHDEPLOY_H

#include <iostream>
#include <cmath>
#include <vector>
#include <mutex>
#include <atomic>
#include <torch/script.h>
#include <torch/torch.h>

#include "Eigen/Dense"
#include "Eigen/Core"

#include "Go2_Enum.h"

using namespace torch::indexing;
using torch::indexing::Slice;

#pragma once
using float32_t = float;
using float64_t = double;


class DeployHigh
{
public:

    DeployHigh();
    ~DeployHigh();

    void Set_IMU(Eigen::VectorXd Accel_, Eigen::VectorXd Gyro_, Eigen::Quaterniond quat);
    void Set_Position(Eigen::VectorXd Robot_Pos_, Eigen::VectorXd Target_Pos_);
    void Set_Observation();
    void Inference();

    Eigen::Vector3d GetAction() { return actions; }

    Eigen::VectorXd Robot_pos = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd Target_pos = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd Error_pos = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd Local_Error_pos = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd Imu_acc = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd Imu_ang_vel = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd Quat = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd Imu_quat = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd Projected_gravity = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd gravity_vec = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd Command = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd action = Eigen::VectorXd::Zero(12); 
    Eigen::VectorXd last_action = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd actions;
    Eigen::Matrix3d Rz_;

    float32_t quat_scale = 1.0;
    float32_t gravity_scale = 1.0;
    float32_t command_lin_scale = 1.5;
    float32_t command_ang_scale = 2.0;
    float32_t action_scale = 1.0;
    float32_t position = 0.3;

    double command_x_vel = 0, command_y_vel = 0, command_yaw_vel = 0, command_height = 0.28;

private:
    torch::jit::script::Module actor;

    torch::DeviceType device;
    std::vector<int64_t> input_dims;
    std::vector<int64_t> input_history_dims;

    torch::Tensor observation_tensor;
    torch::Tensor observation_history_tensor;
    torch::Tensor observation_history_flat;
    
    std::mutex runner_mutex;

    std::atomic<bool> observation_ready = false;

    int obs_dim = 0;
    int num_obs_history = 1;
    int Count = 0;
};

#endif