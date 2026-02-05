#ifndef DEPLOY_H
#define DEPLOY_H

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


class Deploy
{
public:

    Deploy();
    ~Deploy();

    void Set_Body(Eigen::VectorXd lin_vel, Eigen::VectorXd Body_pos_);
    void Set_IMU(Eigen::VectorXd Accel_, Eigen::VectorXd Gyro_, Eigen::Quaterniond quat);
    void Set_Encoder(Eigen::VectorXd q_, Eigen::VectorXd dq_);
    void Set_Joystick(double ly, double lx, double ry, double rx, int ud);
    void Set_Observation();
    void HistoryWrapper(int num_obs_history);
    void HistoryWrapperVel(int num_obs_history);
    void HistoryWrapperMap(int num_obs_history);
    void HistoryWrapperResi(torch::Tensor observation, int num_obs_history);
    void Inference();

    Eigen::Vector3d GetAction(int i) { return actions[i]; }

    Eigen::VectorXd Base_lin_vel = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd Imu_acc = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd Imu_ang_vel = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd Quat = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd Imu_quat = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd Projected_gravity = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd gravity_vec = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd Command = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd action = Eigen::VectorXd::Zero(12); 
    Eigen::VectorXd base_action = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd residual_action = Eigen::VectorXd::Zero(12); 
    Eigen::VectorXd last_action = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd actions[NUM_LEG];
    Eigen::Matrix3d Rz_;
    Eigen::VectorXd default_dof_pos = Eigen::VectorXd::Zero(12);

    float32_t lin_vel_scale = 2.0;
    float32_t ang_vel_scale = 0.25;
    float32_t quat_scale = 1.0;
    float32_t gravity_scale = 1.0;
    float32_t command_lin_scale = 1.5;
    float32_t command_ang_scale = 2.0;
    float32_t action_scale = 1.0;
    float32_t dof_pos_scale = 1.0;
    float32_t dof_vel_scale = 0.05;
    float32_t object_scale = 0.3;

    double command_x_vel = 0, command_y_vel = 0, command_yaw_vel = 0, command_height = 0.28;

private:
    torch::jit::script::Module actor;
    torch::jit::script::Module velocity_estimator;
    torch::jit::script::Module map_encoder;
    torch::jit::script::Module resi_actor;

    torch::DeviceType device;
    std::vector<int64_t> input_dims;
    std::vector<int64_t> input_history_dims;

    torch::Tensor observation_tensor;
    torch::Tensor vel_observation_tensor;
    torch::Tensor map_observation_tensor;
    torch::Tensor resi_observation_tensor;
    
    torch::Tensor observation_history_tensor;
    torch::Tensor vel_observation_history_tensor;
    torch::Tensor map_observation_history_tensor;
    torch::Tensor resi_observation_history_tensor;

    torch::Tensor observation_history_flat;
    torch::Tensor vel_observation_history_flat;
    torch::Tensor map_observation_history_flat;
    torch::Tensor resi_observation_history_flat;
    
    std::mutex runner_mutex;

    std::atomic<bool> observation_ready = false;

    int obs_dim = 0;
    int num_obs_history = 1;
    int Count = 0;
};

#endif