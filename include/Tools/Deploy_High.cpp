#include "Deploy.hpp"

Deploy::Deploy() : device(torch::kCPU)
{
    Robot_pos.setZero(2);
    Target_pos.setZero(2);
    Err_pos.setZero(2);
    Local_Err_pos.setZero(2);
    Imu_acc.setZero(NUM_AXIS);
    Imu_ang_vel.setZero(NUM_AXIS);
    Imu_quat.setZero(4);
    action.setZero(3);
    last_action.setZero(3);
    Projected_gravity.setZero(NUM_AXIS);
    gravity_vec.setZero(NUM_AXIS);
    Rz_.setZero(NUM_AXIS, NUM_AXIS);
    actions.setZero(3);

    gravity_vec << 0, 0, -1;

    try {

        actor = torch::jit::load("/home/unitree/HRR_SM/Go2_Residual_kale/model/high_policy_10000.pt"); 
        actor.to(torch::kCPU);
        actor.eval();

    } catch (const c10::Error& e) {
        std::cerr << "Failed to load model: " << e.what() << std::endl;
    }
}

Deploy::~Deploy() {}

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

void Deploy::Set_Position(Eigen::VectorXd Robot_pos_, Eigen::VectorXd Target_pos_)
{
    Robot_pos = Robot_pos_;
    Target_pos = Target_pos_;

    Eigen::Vector3d Error_Pos3;
    Error_Pos3 << (Target_pos(0) - Robot_pos(0)),
                (Target_pos(1) - Robot_pos(1)),
                0.0;

    Eigen::Quaterniond q(Quat(0), Quat(1), Quat(2), Quat(3));
    q.normalize();

    Eigen::Matrix3d R = q.toRotationMatrix();
    double yaw = std::atan2(R(1,0), R(0,0));

    double ex = Error_Pos3(0);
    double ey = Error_Pos3(1);

    double lx =  std::cos(yaw)*ex + std::sin(yaw)*ey;
    double ly = -std::sin(yaw)*ex + std::cos(yaw)*ey;

    Local_Error_Pos(0) = lx;
    Local_Error_Pos(1) = ly;

    std::cout << "Target_Pos : " << Target_Pos(0) << "  |  " << Target_Pos(1) << std::endl;
}

void Deploy::Set_Observation()
{
    Eigen::Vector3d local_local_error_pos;
    Eigen::Vector3d local_projected_gravity;
    Eigen::VectorXd local_last_action;

    {
        std::lock_guard<std::mutex> lock(runner_mutex);
        local_local_error_pos = Local_Error_Pos;
        local_projected_gravity = Projected_gravity;
        local_default_dof_pos = default_dof_pos;
        local_last_action = last_action;
    }

    std::vector<float> local_error_pos_data(local_local_error_pos.data(), local_local_error_pos.data() + local_local_error_pos.size());
    std::vector<float> projected_gravity_data(local_projected_gravity.data(), local_projected_gravity.data() + local_projected_gravity.size());
    std::vector<float> actions_data(local_last_action.data(), local_last_action.data() + local_last_action.size());

    torch::Tensor local_error_pos_tensor = torch::from_blob(local_error_pos_data.data(), {2}, torch::kFloat).clone() * position;
    torch::Tensor projected_gravity_tensor = torch::from_blob(projected_gravity_data.data(), {3}, torch::kFloat).clone() * gravity_scale;
    torch::Tensor actions_tensor = torch::from_blob(actions_data.data(), {3}, torch::TensorOptions().dtype(torch::kFloat).device(torch::kCPU)).clone() * action_scale;

    auto obs = torch::cat({local_error_pos_tensor,
                           projected_gravity_tensor,
                           actions_tensor   }, 0).contiguous();
    {
        std::lock_guard<std::mutex> lock(runner_mutex);
        observation_tensor = obs;
        observation_ready = true;
    }
}

void Deploy::Inference()
{
    torch::Tensor local_observation_tensor;

    {
        std::lock_guard<std::mutex> lock(runner_mutex);

        local_observation_tensor = observation_tensor;
    }

    std::vector<torch::jit::IValue> inputs;
    torch::Tensor output;

    try 
    {
        inputs.push_back(local_observation_tensor.to(torch::kCPU).to(torch::kFloat));
        output = actor.forward(inputs).toTensor().to(torch::kCPU);

        output = output.clamp(-2, 2);
        output = output.to(torch::kDouble);
        action = Eigen::VectorXd::Map(output.data_ptr<double>(), output.numel());

        last_action = action;

        std::cout << "action : " << last_action(0) << " | " << last_action(1) << " | " << last_action(2) << std::endl;
        {
            std::lock_guard<std::mutex> lock(runner_mutex);
        
            actions[i](X) = action;
            
        }

    }
    catch(const c10::Error& e)
    {
        std::cerr << "Model prediction failed: " << e.what() << std::endl;
    }

}

