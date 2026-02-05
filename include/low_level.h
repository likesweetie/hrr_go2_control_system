#ifndef CUSTOM_H
#define CUSTOM_H

#include <iostream>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <chrono>
#include <iomanip>
#include <queue>
#include <atomic>
#include <lcm/lcm-cpp.hpp>

#include "Eigen/Dense"
// #include "matplotlib-cpp/matplotlibcpp.h"
#include "Joy_Stick.hpp"
#include "Deploy_High.hpp"
#include "Deploy.hpp"
#include "Go2_Enum.h"
#include "shm_utils.hpp"

using namespace unitree::common;
using namespace unitree::robot;
// namespace plt = matplotlibcpp;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_JOYSTICK "rt/wirelesscontroller"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Custom
{
public:
    explicit Custom();
    ~Custom();

    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void JoyStickMessageHandler(const void *message);
    void Command(bool State);
    void Homing_Joint();
    void Encoder_Read();
    void IMU_Read();
    void GPS_Read();
    void FootForce_Read();
    void Joint_Controller();
    void Observation_Update();
    void High_Observation_Update();
    void Run();
    void PolicyRun();
    void HighPolicyRun();
    void PlotRun();
    void JoyRun();
    void DataRun();
    void FSM_Waypoint();

    ShmData* shm_{nullptr};          
    std::uint64_t last_counter_{0};  

    double ucm_x_origin = 0.0f;
    double ucm_y_origin = 0.0f;
    bool ucm_origin_init = false;

    void JoyStick_Control()
    {
        std::lock_guard<std::mutex> lock(joystick_mutex);
        {
            gamepad.KeyUpdate(joystick_msg);
        }

        Stick_LX = gamepad.lx;
        Stick_LY = gamepad.ly;

        Stick_RX = gamepad.rx;
        Stick_RY = gamepad.ry;

        x_vel_command = Stick_LY * 1.;
        y_vel_command = (-1) * (Stick_LX);
        yaw_vel_command = (-1) * (Stick_RX) * 1.57; 

 
        if (gamepad.L1.on_press)
        {
            BL_Button = 1;
        }
        else
        {
            BL_Button = 0;
        }

        if (gamepad.R1.on_press)
        {
            BR_Button = 1;
        }
        else
        {
            BR_Button = 0;
        }

        if (gamepad.L2.on_press)
        {
            Damp_count += 1;
        }

        if (Stick_RY >= 0.3)
        {
            height_command = 1;
        }
        else if (Stick_RY <= -0.3)
        {
            height_command = -1;
        }
        else
        {
            height_command = 0;
        }

        if (gamepad.start.on_press)
        {
            start_count += 1;
        }

        // std::cout << "Run : " << Stick_LY << "   |   " << "Turn : " << Stick_RX << std::endl;
    }

    Eigen::Vector3d Quat2Euler(Eigen::Vector4d quat)
    {
        // quat(0) = w, quat(1) = x, quat(2) = y, quat(3) = z
        double sinr_cosp = 2 * (quat(0) * quat(1) + quat(2) * quat(3));
        double cosr_cosp = 1 - 2 * (quat(1) * quat(1) + quat(2) * quat(2));
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (quat(0) * quat(2) - quat(3) * quat(1));
        double pitch;
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);
        else
            pitch = std::asin(sinp);

        double siny_cosp = 2 * (quat(0) * quat(3) + quat(1) * quat(2));
        double cosy_cosp = 1 - 2 * (quat(2) * quat(2) + quat(3) * quat(3));
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        Eigen::Vector3d rpy(roll, pitch, yaw);
        return rpy;
    }

    double LPF(float x_k, float y_pre, float Ts, float tau_LPF)
    {
        float y_k;
        y_k = (tau_LPF * y_pre + Ts * x_k) / (Ts + tau_LPF);
        return y_k;
    }

    struct Waypoint
    {
        std::string name;
        double x;
        double y;
    };

    // const std::vector<Waypoint> waypoint = {
    //     {"START",  0.0,                  0.0},
    //     {"WP1",   -28.528205533861183, -22.330219073221087},
    //     {"WP2",    -8.866188443964347, -47.973673513159156},
    //     {"WP3",    -1.506497574446257, -42.23294421331957},
    //     {"WP4",   -10.458634274080396, -28.987388744950294},
    //     {"WP5",     9.75069282919867,  -12.969258994795382},
    // };

    const std::vector<Waypoint> waypoint = {
        {"START",   0.0,                0.0},
        {"WP1",     0.0,                10.0},
        {"WP2",    -5.0,               10.0},
        {"WP3",    -10.0,               10.0},
        {"WP4",    -5.0,               0.0},
        {"WP5",    -10.0,               0.0},
    };

    double distanceToTarget(const Eigen::VectorXd& robot, const Waypoint& target)
    {
        return std::sqrt(std::pow(robot(0) - target.x, 2) + std::pow(robot(1) - target.y, 2));
    }

    enum class FSM_Mode
    {
        start_to_way1 = 0,
        way1_to_way2,
        way2_to_way3,
        way3_to_way4,
        way4_to_way5,
        way5_to_start
    };

    Waypoint Current_Target{"", 0.0, 0.0};
    FSM_Mode current_mode = FSM_Mode::way5_to_start;

    Deploy ISSAC;
    DeployHigh HIGH;

    ControlMode controlmode;
    MotorMode motormode;

    Eigen::VectorXd q_ = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd pre_q_ = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd dq_ = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd diff_dq_ = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd filter_dq_ = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd pre_dq_ = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd pre_LPF_dq_ = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd LPF_q_ = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd LPF_dq_ = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd LPF2_dq_ = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd torque = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd action_q = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd Default_Pos = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd target_joint = Eigen::VectorXd::Zero(12);

    Eigen::VectorXd Start_Rad[NUM_LEG];
    Eigen::VectorXd Homing_Rad[NUM_LEG];
    Eigen::VectorXd Start_Pos[NUM_LEG];
    Eigen::VectorXd Homing_Pos[NUM_LEG];
    Eigen::VectorXd Desire_Pos[NUM_LEG];
    Eigen::VectorXd Desire_Vel[NUM_LEG];
    Eigen::VectorXd Via_Pos[NUM_LEG];
    Eigen::VectorXd Error_Pos[NUM_LEG];
    Eigen::VectorXd Error_Vel[NUM_LEG], Filter_Error_Vel[NUM_LEG], Pre_Error_Vel[NUM_LEG];
    Eigen::VectorXd Current_Footpos[NUM_LEG], Ref_Footpos[NUM_LEG], Ref_Footvel[NUM_LEG], Ref_Footacc[NUM_LEG], Target_Footpos[NUM_LEG], Test_Footpos[NUM_LEG];
    Eigen::VectorXd Ref_Angle[NUM_LEG];
    Eigen::Vector3d action_leg[NUM_LEG];
    Eigen::VectorXd torque_[NUM_LEG];

    Eigen::Vector3d High_Command;
    Eigen::Vector3d LPF_High_Command;
    Eigen::Vector3d Pre_High_Command;

    Eigen::VectorXd Robot_Pos = Eigen::VectorXd::Zero(2);
    Eigen::Vector3d Box_Pose;
    Eigen::VectorXd Target_Pos = Eigen::VectorXd::Zero(2);

    Eigen::Vector3d Body_Vel;

    Eigen::Vector4d imu_quat, pino_quat, go2_quaternion, init_quaternion;
    Eigen::Vector3d imu_rpy, imu_rpy_dot;
    Eigen::Matrix3d imu_rot, go2_imu_rot, init_imu_rot;
    Eigen::Vector3d init_imu_rpy, init_imu_rpy_dot, init_imu_lin_acc;
    Eigen::Vector3d go2_imu_rpy, go2_imu_rpy_dot, go2_imu_lin_acc;
    Eigen::Vector3d imu_ang_vel;
    Eigen::Vector3d imu_lin_acc;
    Eigen::Vector3d filtered_imu_lin_acc;
    Eigen::Vector3d Local_imu_rpy_dot;
    Eigen::Matrix3d Rz_;
    Eigen::Quaterniond Quat;

    double Go2_quaternion[4] = {0, 0, 0, 0};
    double Go2_gyroscope[3] = {0, 0, 0};
    double Go2_accelerometer[3] = {0, 0, 0};
    double Go2_rpy[3] = {0, 0, 0};

    Eigen::Quaterniond Body_Quat = Eigen::Quaterniond::Identity();

    Eigen::VectorXd Hip_Pos[NUM_LEG];
    Eigen::VectorXd Step_Position[NUM_LEG];

    double Foot_Force[4] = {0, 0, 0, 0};
    int Foot_Contact[4] = {0, 0, 0, 0};

    double Torque[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double Target_q[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double kd[12] = {0.9, 1.3, 0.7,     0.9, 1.3, 0.7,      0.9, 1.3, 0.7,      0.9, 1.3, 0.7};

    ThreadPtr RunThreadPtr;
    ThreadPtr PlotThreadPtr;
    ThreadPtr PolicyRunThreadPtr;
    ThreadPtr HighPolicyRunThreadPtr;
    ThreadPtr JointThreadPtr;
    ThreadPtr LCMThreadPtr;

    int Start_Flag = 0;
    int Homing_Time = 0;
    int Motion_Time = 0;
    int Policy_time = 0;
    double Plot_Time = 0.0;
    char inputChar = ' ';
    double dt = 0.002;

    double x_vel_command = 0;
    double y_vel_command = 0;
    double x_pos_command = 0;
    double y_pos_command = 0;
    double z_pos_command = 0.32;
    double yaw_command = 0;
    double yaw_vel_command = 0;
    int height_command = 0;

    int BL_Button, BR_Button;
     int UP_Button, DOWN_Button;

    bool Joint_State = false;
    bool Motor_State = false;

    std::vector<double> value_1, value_2, value_3, value_4;
    std::vector<double> time_values;

    std::string filename_1 = "/home/sm/Quadruped/Go2_Reality/src/Log/X_Footpos.txt";
    std::string filename_2 = "/home/sm/Quadruped/Go2_Reality/src/Log/Kalman_X.txt";
    std::string filename_3 = "/home/sm/Quadruped/Go2_Reality/src/Log/Kalman_Y.txt";
    std::string filename_4 = "/home/sm/Quadruped/Go2_Reality/src/Log/Kalman_Z.txt";

    double Stick_LX = 0.0;
    double Stick_LY = 0.0;
    double Stick_RX = 0.0;
    double Stick_RY = 0.0;



protected:
    // publisher
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    // subscriber
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_subscriber;

    unitree_go::msg::dds_::WirelessController_ joystick_msg;
    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    JoyStick gamepad;

    ThreadPtr JoyThreadptr;

    std::mutex joystick_mutex;
    std::mutex Policy_mutex;
    std::mutex Main_mutex;
    std::mutex isaac_mutex;

    int press_count = 0;
    int start_count = 0;
    int Damp_count = 0;
    int yaw_count = 0;
    int hz_count = 0;

    int run_count = 0;
    int Count = 0;
};

#endif
