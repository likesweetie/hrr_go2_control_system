#include "Deploy.hpp"

Deploy::Deploy() : device(torch::kCPU)
{
    Base_lin_vel.setZero(NUM_AXIS);
    Imu_acc.setZero(NUM_AXIS);
    Imu_ang_vel.setZero(NUM_AXIS);
    Imu_quat.setZero(4);
    q.setZero(NUM_DOF);
    dq.setZero(NUM_DOF);
    action.setZero(NUM_DOF);
    last_action.setZero(NUM_DOF);
    Projected_gravity.setZero(NUM_AXIS);
    gravity_vec.setZero(NUM_AXIS);
    Command.setZero(4);
    default_dof_pos.setZero(NUM_DOF);
    Rz_.setZero(NUM_AXIS, NUM_AXIS);

    for (size_t i = 0; i < NUM_LEG; i++)
    {
        actions[i].setZero(NUM_JOINT);
    }

    gravity_vec << 0, 0, -1;
    default_dof_pos << 0.0, 0.685, -1.40,    0.0, 0.685, -1.40,   0.0, 0.685, -1.40,   0.0, 0.685, -1.40;

    Command(3) = 0.28;
    command_height = 0.28;

    try {

        actor = torch::jit::load("/home/unitree/HRR_SM/Go2_Residual/model/model_10400.pt"); 
        actor.to(torch::kCPU);
        actor.eval();

        velocity_estimator = torch::jit::load("/home/unitree/HRR_SM/Go2_Residual/model/velocity_estimator_10400.pt");
        velocity_estimator.to(torch::kCPU); 
        velocity_estimator.eval();

        map_encoder = torch::jit::load("/home/unitree/HRR_SM/Go2_Residual/model/map_encoder_92000.pt"); 
        map_encoder.to(torch::kCPU);
        map_encoder.eval();

        resi_actor = torch::jit::load("/home/unitree/HRR_SM/Go2_Residual/model/model_92000.pt"); 
        resi_actor.to(torch::kCPU);
        resi_actor.eval();

    } catch (const c10::Error& e) {
        std::cerr << "Failed to load model: " << e.what() << std::endl;
    }
}

Deploy::~Deploy() {}

void Deploy::Set_Body(Eigen::VectorXd lin_vel, Eigen::VectorXd Body_pos_)
{
    std::lock_guard<std::mutex> lock(runner_mutex);
   
    if (!lin_vel.array().isNaN().any()) 
    {
        Base_lin_vel = lin_vel;
    }
    else
    {
        std::cerr << "lin vel is NaN!!!" << std::endl;
    }
}

void Deploy::Set_IMU(Eigen::VectorXd Accel_, Eigen::VectorXd Gyro_, Eigen::Quaterniond quat)
{
    std::lock_guard<std::mutex> lock(runner_mutex);

    if (!Accel_.array().isNaN().any()) 
    {
        Imu_acc = Accel_;
    }
    else
    {
        std::cerr << "Imu acc is NaN!!!" << std::endl;
    }

    if (!Gyro_.array().isNaN().any()) 
    {
        Imu_ang_vel = Gyro_;
    }
    else
    {
        std::cerr << "Imu ang vel is NaN!!!" << std::endl;
    }

    if (!quat.coeffs().array().isNaN().any()) 
    {
        quat.Eigen::Quaterniond::normalize();
        Quat = quat.coeffs();
        Imu_quat(0) = Quat(1);
        Imu_quat(1) = Quat(2);
        Imu_quat(2) = Quat(3);
        Imu_quat(3) = Quat(0);
        Rz_ = quat.toRotationMatrix();
        // Projected_gravity = Rz_.transpose() * gravity_vec;
        Projected_gravity = quat.inverse() * gravity_vec;
    }
    else
    {
        std::cerr << "Imu quat is NaN!!!" << std::endl;
    }

    // std::cout << "Imu_ang : " << Imu_ang_vel(0) << " | " << Imu_ang_vel(1) << " | " << Imu_ang_vel(2) << std::endl;
    // std::cout << "gravity : " << Projected_gravity(0) << " | " << Projected_gravity(1) << " | " << Projected_gravity(2) << std::endl;
}

void Deploy::Set_Encoder(Eigen::VectorXd q_, Eigen::VectorXd dq_)
{
    std::lock_guard<std::mutex> lock(runner_mutex);
    
    if (!q_.array().isNaN().any()) 
    {
        q = q_;
    }
    else
    {
        std::cerr << "q is NaN!!!" << std::endl;
    }

    if (!dq_.array().isNaN().any()) 
    {
        dq = dq_;
    }
    else
    {
        std::cerr << "dq is NaN!!!" << std::endl;
    }

    // std::cout << "q : " << q(0) << " | " << q(1) << " | " << q(2) << std::endl;
    // std::cout << "dq : " << dq(0) << " | " << dq(1) << " | " << dq(2) << std::endl;
}

void Deploy::Set_Joystick(double ly, double lx, double ry, double rx, int ud)
{
    std::lock_guard<std::mutex> lock(runner_mutex);

    if (!std::isnan(ly)) 
    {
        if(std::abs(ly) < 0.2)
        {
            command_x_vel = 0.0;
        }
        else
        {
            command_x_vel = ly;
        }
    }
    else
    {
        std::cerr << "command_x_vel is NaN!!!" << std::endl;
    }


    if (!std::isnan(lx)) 
    {
        if(std::abs(lx) < 0.2)
        {
            command_y_vel = 0.0;
        }
        else
        {
            command_y_vel = lx;
        }
    }
    else
    {
        std::cerr << "command_y_vel is NaN!!!" << std::endl;
    }


    if (!std::isnan(rx)) 
    {
        if(std::abs(rx) < 0.2)
        {
            command_yaw_vel = 0.0;
        }
        else
        {
            command_yaw_vel = rx;
        }
    }
    else
    {
        std::cerr << "command_yaw_vel is NaN!!!" << std::endl;
    }

    // if (!std::isnan(ud)) 
    // {
    //     if(ud == 1)
    //     {
    //         command_height += 0.0005;
    //         if(command_height >= 0.35)
    //         {
    //             command_height = 0.35;
    //         }
    //     }
    //     else if(ud == -1)
    //     {
    //         command_height -= 0.0005;
    //         if(command_height <= 0.2)
    //         {
    //             command_height = 0.2;
    //         }
    //     }
    // }
    // else
    // {
    //     std::cerr << "command_yaw_vel is NaN!!!" << std::endl;
    // }

    Command(0) = command_x_vel * 0.5;
    Command(1) = command_y_vel * 0.5;
    Command(2) = command_yaw_vel * 0.5;
    Command(3) = 0.28;
    // Command(4) = 0.0;
    // Command(5) = 0.0;
    // Command(6) = 0.0;

    // if(std::abs(command_x_vel) >= 0.1 || std::abs(command_y_vel) >= 0.1 || std::abs(command_yaw_vel) >= 0.1)
    // {
    //     Command(7) = 1;
    // }
    // else
    // {
    //     Command(7) = 0;
    // }

    // std::cout << "command : " << Command(0) << " | " << Command(1) << " | " << Command(2) << "  |  " << command_height << std::endl;
}

void Deploy::Set_Observation()
{
    Eigen::Vector3d local_lin_vel;
    Eigen::Vector3d local_ang_vel, local_projected_gravity;
    Eigen::VectorXd local_command;
    Eigen::Vector4d local_quat;
    Eigen::VectorXd local_dof_pos, local_dof_vel, local_default_dof_pos, local_last_action;

    {
        std::lock_guard<std::mutex> lock(runner_mutex);
        local_lin_vel = Base_lin_vel;
        local_ang_vel = Imu_ang_vel;
        local_quat = Imu_quat;
        local_projected_gravity = Projected_gravity;
        local_command = Command;
        local_dof_pos = q;
        local_dof_vel = dq;
        local_default_dof_pos = default_dof_pos;
        local_last_action = last_action;
    }

    std::vector<float> lin_vel_data(Base_lin_vel.data(), Base_lin_vel.data() + Base_lin_vel.size());
    std::vector<float> quat_data(local_quat.data(), local_quat.data() + local_quat.size());
    std::vector<float> ang_vel_data(local_ang_vel.data(), local_ang_vel.data() + local_ang_vel.size());
    std::vector<float> projected_gravity_data(local_projected_gravity.data(), local_projected_gravity.data() + local_projected_gravity.size());
    std::vector<float> command_data(local_command.data(), local_command.data() + local_command.size());

    Eigen::VectorXf dof_pos_adjusted = (local_dof_pos - local_default_dof_pos).cast<float>();
    std::vector<float> dof_pos_data(dof_pos_adjusted.data(), dof_pos_adjusted.data() + dof_pos_adjusted.size());
    std::vector<float> dof_vel_data(local_dof_vel.data(), local_dof_vel.data() + local_dof_vel.size());
    std::vector<float> actions_data(local_last_action.data(), local_last_action.data() + local_last_action.size());

    // float32 데이터로부터 텐서 생성
    torch::Tensor lin_vel_tensor = torch::from_blob(lin_vel_data.data(), {NUM_AXIS}, torch::TensorOptions().dtype(torch::kFloat).device(torch::kCPU)).clone() * lin_vel_scale;
    torch::Tensor ang_vel_tensor = torch::from_blob(ang_vel_data.data(), {NUM_AXIS}, torch::TensorOptions().dtype(torch::kFloat).device(torch::kCPU)).clone() * ang_vel_scale;
    torch::Tensor quat_tensor = torch::from_blob(quat_data.data(), {4}, torch::TensorOptions().dtype(torch::kFloat).device(torch::kCPU)).clone() * quat_scale;
    torch::Tensor projected_gravity_tensor = torch::from_blob(projected_gravity_data.data(), {NUM_AXIS}, torch::kFloat).clone() * gravity_scale;
    torch::Tensor command_tensor = torch::from_blob(command_data.data(), {4}, torch::TensorOptions().dtype(torch::kFloat).device(torch::kCPU)).clone();
    torch::Tensor dof_pos_tensor = torch::from_blob(dof_pos_data.data(), {NUM_DOF}, torch::TensorOptions().dtype(torch::kFloat).device(torch::kCPU)).clone() * dof_pos_scale;
    torch::Tensor dof_vel_tensor = torch::from_blob(dof_vel_data.data(), {NUM_DOF}, torch::TensorOptions().dtype(torch::kFloat).device(torch::kCPU)).clone() * dof_vel_scale;
    torch::Tensor actions_tensor = torch::from_blob(actions_data.data(), {NUM_DOF}, torch::TensorOptions().dtype(torch::kFloat).device(torch::kCPU)).clone() * action_scale;

    auto obs = torch::cat({ang_vel_tensor,
                           projected_gravity_tensor,
                           command_tensor,
                           dof_pos_tensor,
                           dof_vel_tensor,
                           actions_tensor}, 0).contiguous();
    {
        std::lock_guard<std::mutex> lock(runner_mutex);
        observation_tensor = obs;
        vel_observation_tensor = obs;
        map_observation_tensor = obs;
        resi_observation_tensor = obs;
        observation_ready = true;
    }
    // observation_tensor = torch::cat({   ang_vel_tensor,
    //                                     projected_gravity_tensor,
    //                                     command_tensor,
    //                                     dof_pos_tensor,
    //                                     dof_vel_tensor,
    //                                     actions_tensor,  }, 0);

    // vel_observation_tensor = torch::cat({   ang_vel_tensor,
    //                                         projected_gravity_tensor,
    //                                         command_tensor,
    //                                         dof_pos_tensor,
    //                                         dof_vel_tensor,
    //                                         actions_tensor,  }, 0);

    // map_observation_tensor = torch::cat({   ang_vel_tensor,
    //                                         projected_gravity_tensor,
    //                                         command_tensor,
    //                                         dof_pos_tensor,
    //                                         dof_vel_tensor,
    //                                         actions_tensor,  }, 0);

    // resi_observation_tensor = torch::cat({  ang_vel_tensor,
    //                                         projected_gravity_tensor,
    //                                         command_tensor,
    //                                         dof_pos_tensor,
    //                                         dof_vel_tensor,
    //                                         actions_tensor,  }, 0);

    // observation_ready = true;
}

void Deploy::HistoryWrapper(int num_obs_history)
{
    auto obs = observation_tensor.reshape(-1).contiguous();
    const int64_t D = obs.size(0);

    const bool need_realloc = 
        !observation_history_tensor.defined() ||
        observation_history_tensor.size(0) != num_obs_history ||
        observation_history_tensor.size(1) != D ||
        observation_history_tensor.scalar_type() != obs.scalar_type() ||
        observation_history_tensor.device() != obs.device();

    if(need_realloc)
    {
        observation_history_tensor = torch::zeros({num_obs_history, D}, obs.options());
        this->num_obs_history = num_obs_history;
        this->obs_dim = static_cast<int>(D);
    }

    if(num_obs_history == 1)
    {
        observation_history_tensor[0].copy_(obs);
    }
    else
    {
        auto rhs = observation_history_tensor.index({Slice(1, c10::nullopt)}).clone();
        observation_history_tensor.index_put_({Slice(0, num_obs_history - 1)}, rhs);
        observation_history_tensor.index_put_({num_obs_history - 1}, obs);
    }

    observation_history_flat = observation_history_tensor.reshape({1, num_obs_history * D}).contiguous();
}

void Deploy::HistoryWrapperVel(int num_obs_history)
{
    auto obs = vel_observation_tensor.reshape(-1).contiguous();
    const int64_t D = obs.size(0);

    const bool need_realloc = 
        !vel_observation_history_tensor.defined() ||
        vel_observation_history_tensor.size(0) != num_obs_history ||
        vel_observation_history_tensor.size(1) != D ||
        vel_observation_history_tensor.scalar_type() != obs.scalar_type() ||
        vel_observation_history_tensor.device() != obs.device();

    if(need_realloc)
    {
        vel_observation_history_tensor = torch::zeros({num_obs_history, D}, obs.options());
        this->num_obs_history = num_obs_history;
        this->obs_dim = static_cast<int>(D);
    }

    if(num_obs_history == 1)
    {
        vel_observation_history_tensor[0].copy_(obs);
    }
    else
    {
        auto rhs = vel_observation_history_tensor.index({Slice(1, c10::nullopt)}).clone();
        vel_observation_history_tensor.index_put_({Slice(0, num_obs_history - 1)}, rhs);
        vel_observation_history_tensor.index_put_({num_obs_history - 1}, obs);
    }

    vel_observation_history_flat = vel_observation_history_tensor.reshape({1, num_obs_history * D}).contiguous();
}

void Deploy::HistoryWrapperMap(int num_obs_history)
{
    auto obs = map_observation_tensor.reshape(-1).contiguous();
    const int64_t D = obs.size(0);

    const bool need_realloc = 
        !map_observation_history_tensor.defined() ||
        map_observation_history_tensor.size(0) != num_obs_history ||
        map_observation_history_tensor.size(1) != D ||
        map_observation_history_tensor.scalar_type() != obs.scalar_type() ||
        map_observation_history_tensor.device() != obs.device();

    if(need_realloc)
    {
        map_observation_history_tensor = torch::zeros({num_obs_history, D}, obs.options());
        this->num_obs_history = num_obs_history;
        this->obs_dim = static_cast<int>(D);
    }

    if(num_obs_history == 1)
    {
        map_observation_history_tensor[0].copy_(obs);
    }
    else
    {
        auto rhs = map_observation_history_tensor.index({Slice(1, c10::nullopt)}).clone();
        map_observation_history_tensor.index_put_({Slice(0, num_obs_history - 1)}, rhs);
        map_observation_history_tensor.index_put_({num_obs_history - 1}, obs);
    }

    map_observation_history_flat = map_observation_history_tensor.reshape({1, num_obs_history * D}).contiguous();
}

void Deploy::HistoryWrapperResi(torch::Tensor observation, int num_obs_history)
{
    auto obs = observation.reshape(-1).contiguous();
    const int64_t D = obs.size(0);

    const bool need_realloc = 
        !resi_observation_history_tensor.defined() ||
        resi_observation_history_tensor.size(0) != num_obs_history ||
        resi_observation_history_tensor.size(1) != D ||
        resi_observation_history_tensor.scalar_type() != obs.scalar_type() ||
        resi_observation_history_tensor.device() != obs.device();

    if(need_realloc)
    {
        resi_observation_history_tensor = torch::zeros({num_obs_history, D}, obs.options());
        this->num_obs_history = num_obs_history;
        this->obs_dim = static_cast<int>(D);
    }

    if(num_obs_history == 1)
    {
        resi_observation_history_tensor[0].copy_(obs);
    }
    else
    {
        auto rhs = resi_observation_history_tensor.index({Slice(1, c10::nullopt)}).clone();
        resi_observation_history_tensor.index_put_({Slice(0, num_obs_history - 1)}, rhs);
        resi_observation_history_tensor.index_put_({num_obs_history - 1}, obs);
    }

    resi_observation_history_flat = resi_observation_history_tensor.reshape({1, num_obs_history * D}).contiguous();
}

void Deploy::Inference()
{
    torch::Tensor local_observation_tensor, local_vel_observation_tensor, local_map_observation_tensor, local_resi_observation_tensor, local_resi_observation_tensor_flat;

    // HistoryWrapper(5);
    // HistoryWrapperVel(5);

    {
        std::lock_guard<std::mutex> lock(runner_mutex);

        HistoryWrapper(5);
        HistoryWrapperVel(5);

        local_observation_tensor = observation_history_flat;
        local_vel_observation_tensor = vel_observation_history_flat;
        local_resi_observation_tensor = resi_observation_tensor;
    }

    std::vector<torch::jit::IValue> inputs, vel_inputs, map_inputs, resi_inputs;
    std::vector<torch::jit::IValue> final_inputs;

    torch::Tensor vel_latent, base_output, output;

    try 
    {
        vel_inputs.push_back(local_vel_observation_tensor.to(torch::kCPU).to(torch::kFloat));
        // torch::Tensor vel_latent = velocity_estimator.forward(vel_inputs).toTensor().to(torch::kCPU);
        vel_latent = velocity_estimator.forward(vel_inputs).toTensor().to(torch::kCPU);

        torch::Tensor combined = torch::cat({local_observation_tensor, vel_latent}, 1);
        inputs.push_back(combined.to(torch::kCPU).to(torch::kFloat));
        // torch::Tensor base_output = actor.forward(inputs).toTensor().to(torch::kCPU);
        base_output = actor.forward(inputs).toTensor().to(torch::kCPU);

        torch::Tensor base_output_flat = base_output.reshape({1, -1}).contiguous();
        torch::Tensor resi_observation_tensor_flat = local_resi_observation_tensor.reshape({1, -1}).contiguous();

        torch::Tensor resi_combined = torch::cat({resi_observation_tensor_flat, base_output_flat}, 1);

        // HistoryWrapperMap(5);
        // HistoryWrapperResi(resi_combined, 5);

        torch::Tensor local_map_hist_flat;
        torch::Tensor local_resi_hist_flat;

        {
            std::lock_guard<std::mutex> lock(runner_mutex);

            HistoryWrapperMap(5);
            HistoryWrapperResi(resi_combined, 5);
            
            local_map_observation_tensor = map_observation_history_flat;
            local_resi_observation_tensor_flat = resi_observation_history_flat;
        }

        map_inputs.push_back(local_map_observation_tensor.to(torch::kCPU).to(torch::kFloat));
        torch::Tensor map_latent = map_encoder.forward(map_inputs).toTensor().to(torch::kCPU);

        torch::Tensor final_combined = torch::cat({local_resi_observation_tensor_flat, map_latent}, 1);
        final_inputs.push_back(final_combined.to(torch::kCPU).to(torch::kFloat));
        torch::Tensor output = resi_actor.forward(final_inputs).toTensor().to(torch::kCPU);

        // base_output = base_output.clamp(-10, 10);
        // base_output = base_output.to(torch::kDouble);
        // base_action = Eigen::VectorXd::Map(base_output.data_ptr<double>(), base_output.numel());

        base_output = base_output.clamp(-10, 10).to(torch::kCPU).to(torch::kDouble).contiguous();
        auto base_flat = base_output.reshape({-1}).contiguous();
        base_action = Eigen::VectorXd::Map(base_flat.data_ptr<double>(), base_flat.numel());

        output = output.clamp(-10, 10);
        output = output.to(torch::kDouble);
        action = Eigen::VectorXd::Map(output.data_ptr<double>(), output.numel());

        last_action = base_action;

        // std::cout << "action : " << last_action(0) << " | " << last_action(1) << " | " << last_action(2) << std::endl;
        {
            std::lock_guard<std::mutex> lock(runner_mutex);
        
            for(int i = 0; i < NUM_LEG; i++)
            {
                actions[i](X) = base_action(3 * i + 0) + action(3 * i + 0);
                actions[i](Y) = base_action(3 * i + 1) + action(3 * i + 1);
                actions[i](Z) = base_action(3 * i + 2) + action(3 * i + 2);
            }
        }

    }
    catch(const c10::Error& e)
    {
        std::cerr << "Model prediction failed: " << e.what() << std::endl;
    }

}

