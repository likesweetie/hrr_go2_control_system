#include "low_level.h"

Custom::Custom()
{
    go2_imu_rpy_dot.setZero(NUM_AXIS);
    go2_imu_rpy.setZero(NUM_AXIS);
    go2_imu_lin_acc.setZero(NUM_AXIS);
    Quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

    for (size_t i = 0; i < NUM_LEG; i++)
    {
        torque_[i].setZero(NUM_AXIS);
        Start_Rad[i].setZero(NUM_AXIS);
        Homing_Rad[i].setZero(NUM_AXIS);
        Start_Pos[i].setZero(NUM_AXIS);
        Homing_Pos[i].setZero(NUM_AXIS);
        Desire_Pos[i].setZero(NUM_AXIS);
        Desire_Vel[i].setZero(NUM_AXIS);
        Via_Pos[i].setZero(NUM_AXIS);
        Error_Pos[i].setZero(NUM_AXIS);
        Error_Vel[i].setZero(NUM_AXIS);
        action_leg[i].setZero(NUM_AXIS);
    }
    Start_Rad[FL] << 0.0, 1.36, -2.65;
    Start_Rad[FR] << 0.0, 1.36, -2.65;
    Start_Rad[RL] << 0.0, 1.36, -2.65;
    Start_Rad[RR] << 0.0, 1.36, -2.65;

    Homing_Rad[FL] << 0.0, 0.685, -1.40;
    Homing_Rad[FR] << -0.0, 0.685, -1.40;
    Homing_Rad[RL] << 0.0, 0.685, -1.40;
    Homing_Rad[RR] << -0.0, 0.685, -1.40;

    Default_Pos << 0.0, 0.685, -1.40,    -0.0, 0.685, -1.40,   0.0, 0.685, -1.40,   -0.0, 0.685, -1.40;

    controlmode = INIT;
    motormode = M_HOMING;

    current_mode = FSM_Mode::way5_to_start;

    // Target_Pos(0) = 471419.869724;
    // Target_Pos(1) = 4167740.629269;

}

Custom::~Custom()
{
}

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void signalHandler(int signum)
{
    // std::cout << "Interrupt signal (" << signum << ") received.\n";

    // plt::close();

    // exit(signum);
}

void Custom::Init()
{
    const char* shm_name = std::getenv("SHM_NAME");
    if (!shm_name) shm_name = "/shm0";

    shm_ = shm_utils::OpenShm(shm_name);
    if (!shm_) {
        std::cerr << "[ERROR] OpenShm failed: " << shm_name << std::endl;
    }

    InitLowCmd();

    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    joystick_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    joystick_subscriber->InitChannel(std::bind(&Custom::JoyStickMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread*/
    RunThreadPtr = CreateRecurrentThreadEx("motionrun", UT_CPU_ID_NONE, 2000, &Custom::Run, this);
    HighPolicyRunThreadPtr = CreateRecurrentThreadEx("highpolicyrun", UT_CPU_ID_NONE, 20000, &Custom::HighPolicyRun, this);
    PolicyRunThreadPtr = CreateRecurrentThreadEx("policyrun", UT_CPU_ID_NONE, 20000, &Custom::PolicyRun, this);
    // PlotThreadPtr = CreateRecurrentThreadEx("plotrun", UT_CPU_ID_NONE, 2000, &Custom::PlotRun, this);
    JoyThreadptr = CreateRecurrentThreadEx("joyrun", UT_CPU_ID_NONE, 2000, &Custom::JoyRun, this);
}

void Custom::LoadConfig(const std::string& yaml_path)
{
    try {
        YAML::Node cfg = YAML::LoadFile(yaml_path);

        // waypoint.use_yaml 체크
        if (!cfg["waypoint"] || !cfg["waypoint"]["use_yaml"]) {
            std::cerr << "[Waypoint] waypoint.use_yaml not found\n";
            return;
        }

        bool use_yaml = cfg["waypoint"]["use_yaml"].as<bool>();
        if (!use_yaml) {
            std::cerr << "[Waypoint] use_yaml == false\n";
            return;
        }

        // waypoint.points 체크
        if (!cfg["waypoint"]["points"] || !cfg["waypoint"]["points"].IsSequence()) {
            std::cerr << "[Waypoint] waypoint.points not found or not sequence\n";
            return;
        }

        waypoint.clear();

        const YAML::Node& points = cfg["waypoint"]["points"];
        for (std::size_t i = 0; i < points.size(); ++i) {
            const YAML::Node& p = points[i];

            if (!p.IsSequence() || p.size() != 3) {
                std::cerr << "[Waypoint] Invalid waypoint format at index "
                          << i << "\n";
                continue;
            }

            Waypoint wp;
            wp.name = p[0].as<std::string>();
            wp.x    = p[1].as<double>();
            wp.y    = p[2].as<double>();

            waypoint.push_back(wp);
        }

        std::cout << "[Waypoint] Loaded " << waypoint.size()
                  << " waypoints\n";

    } catch (const YAML::Exception& e) {
        std::cerr << "[YAML ERROR] " << e.what() << "\n";
    }
}



void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;    
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void Custom::LowStateMessageHandler(const void *message)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
}

void Custom::JoyStickMessageHandler(const void *message)
{
    std::lock_guard<std::mutex> lock(joystick_mutex);
    joystick_msg = *(unitree_go::msg::dds_::WirelessController_ *)message;
}

void Custom::Command(bool State)
{
    Rz_ = Quat.toRotationMatrix();

    if (Joint_State == true)
    {
        switch (controlmode)
        {
        case INIT:
        {
            controlmode = HOMING;
            break;
        }
        case HOMING:
        {
            Homing_Joint();
            break;
        }
        case WALKING:
        {
            FSM_Waypoint();

            if (!ucm_origin_init)
            {
                std::cout << "ucm origin set" << std::endl;
                ucm_x_origin = shm_->gps_data.ucm_x* 1.0f;
                ucm_y_origin = shm_->gps_data.ucm_y* 1.0f;
                std::cout << ucm_x_origin* 1.0f << "  |  " << ucm_y_origin* 1.0f << "  |  " << std::endl;
                ucm_origin_init = true;

            }
                

            if(start_count > 2)
            {
                
                for (int i = 0; i < NUM_LEG; i++)
                {
                    if (std::isnan(action_leg[i](X)) || std::isnan(action_leg[i](Y)) || std::isnan(action_leg[i](Z)))
                    {
                        std::cout << "Error NAN Action" << std::endl;
                        Damp_count++;

                        return;
                    }

                    action_q(3 * i + 0) = action_leg[i](X);  
                    action_q(3 * i + 1) = action_leg[i](Y);
                    action_q(3 * i + 2) = action_leg[i](Z);
                }

                // std::cout << action_q(0) << "  |  " << action_q(1) << "  |  " <<  action_q(2) << std::endl;
                // std::cout << action_q(3) << "  |  " << action_q(4) << "  |  " <<  action_q(5) << std::endl;
                // std::cout << action_q(6) << "  |  " << action_q(7) << "  |  " <<  action_q(8) << std::endl;
                // std::cout << action_q(9) << "  |  " << action_q(10) << "  |  " <<  action_q(11) << std::endl;

                for (int i = 0; i < 12; i++)
                {
                    target_joint(i) = action_q(i) * 0.25;
                    target_joint(i) += Default_Pos(i);
                    // target_joint(i) = Default_Pos(i);
                }

                // std::cout << "Action!!!!!!!" << std::endl;
            }
            else
            {
                for (int i = 0; i < 12; i++)
                {
                    target_joint(i) = Default_Pos(i);
                }
            }

            Target_q[0] = target_joint(3);
            Target_q[1] = target_joint(4);
            Target_q[2] = target_joint(5);

            Target_q[3] = target_joint(0);
            Target_q[4] = target_joint(1);
            Target_q[5] = target_joint(2);

            Target_q[6] = target_joint(9);
            Target_q[7] = target_joint(10);
            Target_q[8] = target_joint(11);

            Target_q[9] = target_joint(6);
            Target_q[10] = target_joint(7);
            Target_q[11] = target_joint(8);

            break;
        }
        }
    }

    // DataRun();
}

void Custom::Homing_Joint()
{
    if (Motion_Time > 500)
    {
        if (Start_Flag == 0)
        {
            // Motor_State = true;
            // Start_Flag = 1;

            std::cout << "Press 'R' to start moving to the initial stance..." << std::endl;
            std::cin >> inputChar;

            if (inputChar == 'R' || inputChar == 'r')
            {
                std::cout << "Start  !!!" << std::endl;
                

                Via_Pos[FL] << q_(0), q_(1), q_(2);
                Via_Pos[FR] << q_(3), q_(4), q_(5);
                Via_Pos[RL] << q_(6), q_(7), q_(8);
                Via_Pos[RR] << q_(9), q_(10), q_(11);

                Motor_State = true;
                Start_Flag = 1;
            }
        }
        if (Start_Flag == 1)
        {
            for (int leg = 0; leg < NUM_LEG; leg++)
            {
                Desire_Pos[leg] = Via_Pos[leg] + (Start_Rad[leg] - Via_Pos[leg]) * 0.5 * (1 - cos(M_PI * (Homing_Time) / 1000));
            }

            if (Homing_Time < 1000)
            {
                Homing_Time++;
            }
            else if (Homing_Time == 1000)
            {
                Homing_Time = 0;
                Start_Flag = 2;

                Via_Pos[FL] << q_(0), q_(1), q_(2);
                Via_Pos[FR] << q_(3), q_(4), q_(5);
                Via_Pos[RL] << q_(6), q_(7), q_(8);
                Via_Pos[RR] << q_(9), q_(10), q_(11);
            }
        }
        if (Start_Flag == 2)
        {
            for (int leg = 0; leg < NUM_LEG; leg++)
            {
                Desire_Pos[leg] = Via_Pos[leg] + (Homing_Rad[leg] - Via_Pos[leg]) * 0.5 * (1 - cos(M_PI * (Homing_Time) / 1000));
            }

            if (Homing_Time < 1000)
            {
                Homing_Time++;
            }
            else if (Homing_Time == 1000)
            {
                Homing_Time = 1000;
                Start_Flag = 2;
                controlmode = WALKING;
                motormode = NORMAL;  
            }
        }
    }

    double Task_kp = 70.0;
    double Task_kd = 0.0;

    Error_Pos[FL](0) = (Desire_Pos[FL](0) - q_(0));
    Error_Pos[FL](1) = (Desire_Pos[FL](1) - q_(1));
    Error_Pos[FL](2) = (Desire_Pos[FL](2) - q_(2));

    Error_Pos[FR](0) = (Desire_Pos[FR](0) - q_(3));
    Error_Pos[FR](1) = (Desire_Pos[FR](1) - q_(4));
    Error_Pos[FR](2) = (Desire_Pos[FR](2) - q_(5));

    Error_Pos[RL](0) = (Desire_Pos[RL](0) - q_(6));
    Error_Pos[RL](1) = (Desire_Pos[RL](1) - q_(7));
    Error_Pos[RL](2) = (Desire_Pos[RL](2) - q_(8));

    Error_Pos[RR](0) = (Desire_Pos[RR](0) - q_(9));
    Error_Pos[RR](1) = (Desire_Pos[RR](1) - q_(10));
    Error_Pos[RR](2) = (Desire_Pos[RR](2) - q_(11));

    for (int leg = 0; leg < NUM_LEG; leg++)
    {
        torque_[leg] = (Task_kp * Error_Pos[leg]);
        torque_[leg] = torque_[leg].cwiseMax(Eigen::VectorXd::Constant(3, -20)).cwiseMin(Eigen::VectorXd::Constant(3, 20));
    }

    Torque[0] = torque_[FR](0);
    Torque[1] = torque_[FR](1);
    Torque[2] = torque_[FR](2);

    Torque[3] = torque_[FL](0);
    Torque[4] = torque_[FL](1);
    Torque[5] = torque_[FL](2);

    Torque[6] = torque_[RR](0);
    Torque[7] = torque_[RR](1);
    Torque[8] = torque_[RR](2);

    Torque[9] = torque_[RL](0);
    Torque[10] = torque_[RL](1);
    Torque[11] = torque_[RL](2);
}

void Custom::Encoder_Read()
{
    for (int i = 0; i < 12; i++)
    {
        q_(i) = low_state.motor_state()[i].q();
        dq_(i) = low_state.motor_state()[i].dq();
    }

    Eigen::VectorXd temp_q_ = q_;
    Eigen::VectorXd temp_dq_ = dq_;

    q_.segment<3>(0) = temp_q_.segment<3>(3);
    q_.segment<3>(3) = temp_q_.segment<3>(0);
    q_.segment<3>(6) = temp_q_.segment<3>(9);
    q_.segment<3>(9) = temp_q_.segment<3>(6);

    dq_.segment<3>(0) = temp_dq_.segment<3>(3);
    dq_.segment<3>(3) = temp_dq_.segment<3>(0);
    dq_.segment<3>(6) = temp_dq_.segment<3>(9);
    dq_.segment<3>(9) = temp_dq_.segment<3>(6);

    Joint_State = true;
}

void Custom::IMU_Read()
{
    for (int i = 0; i < 4; i++)
    {
        Go2_quaternion[i] = low_state.imu_state().quaternion()[i];

        if (Motion_Time < 1000)
        {
            init_quaternion(0) = Go2_quaternion[0];
            init_quaternion(1) = Go2_quaternion[1];
            init_quaternion(2) = Go2_quaternion[2];
            init_quaternion(3) = Go2_quaternion[3];
        }
        else if (Motion_Time >= 1000)
        {
            go2_quaternion(0) = Go2_quaternion[0];
            go2_quaternion(1) = Go2_quaternion[1];
            go2_quaternion(2) = Go2_quaternion[2];
            go2_quaternion(3) = Go2_quaternion[3];
        }

        Eigen::Vector4d quaternion(go2_quaternion[0], go2_quaternion[1], go2_quaternion[2], go2_quaternion[3]);
        imu_rpy = Quat2Euler(quaternion);
    }

    for (int i = 0; i < 3; i++)
    {
        Go2_gyroscope[i] = low_state.imu_state().gyroscope()[i];
        imu_rpy_dot(i) = Go2_gyroscope[i];
    }
    for (int i = 0; i < 3; i++)
    {
        Go2_accelerometer[i] = low_state.imu_state().accelerometer()[i];
        imu_lin_acc(i) = Go2_accelerometer[i];
    }
    for (int i = 0; i < 3; i++)
    {
        Go2_rpy[i] = low_state.imu_state().rpy()[i];
        imu_rpy(i) = Go2_rpy[i];   
    }

    Quat.w() = go2_quaternion[0];
    Quat.x() = go2_quaternion[1];
    Quat.y() = go2_quaternion[2];
    Quat.z() = go2_quaternion[3];

    // std::cout << go2_quaternion[0] << "  |  " << go2_quaternion[1] << "  |  " << go2_quaternion[2] << "  |  " << go2_quaternion[3] << std::endl;

    if (Motion_Time < 1000)
    {
        init_imu_rpy = imu_rpy;
        init_imu_rpy_dot = imu_rpy_dot;
        init_imu_lin_acc = imu_lin_acc;
        init_imu_rot = imu_rot;
    }
    else if (Motion_Time >= 1000)
    {
        go2_imu_rpy(0) = imu_rpy(0);
        go2_imu_rpy(1) = imu_rpy(1);
        go2_imu_rpy(2) = imu_rpy(2) - init_imu_rpy(2);

        go2_imu_rpy_dot(0) = imu_rpy_dot(0);
        go2_imu_rpy_dot(1) = imu_rpy_dot(1);
        go2_imu_rpy_dot(2) = imu_rpy_dot(2) - init_imu_rpy_dot(2);

        go2_imu_lin_acc(0) = imu_lin_acc(0);
        go2_imu_lin_acc(1) = imu_lin_acc(1);
        go2_imu_lin_acc(2) = imu_lin_acc(2);
    }
}

void Custom::GPS_Read()
{
    if (!shm_) return;


    const GpsData& gps = shm_->gps_data;

    // Robot_Pos에 저장 (x=lat, y=lon, z=alt)
    
    if (ucm_origin_init)
    {
        Robot_Pos(0) = (gps.ucm_x - ucm_x_origin) * 1.0f;
        Robot_Pos(1) = (gps.ucm_y - ucm_y_origin)* 1.0f;
        // Robot_Pos(2) = 0.f;
    }
    else
    {
        Robot_Pos(0) = 0.f;
        Robot_Pos(1) = 0.f;

        for (auto& wp :shifted_waypoint)
        {
            wp.x - gps.ucm_x;
            wp.y - gps.ucm_y;
        }

        
        // Robot_Pos(2) = 0.f;
    }

    // std::cout << Robot_Pos(0) << "  |  " << Robot_Pos(1) << "  |  " << std::endl;


}

void Custom::FootForce_Read()
{
    Foot_Force[FL] = low_state.foot_force()[FR];
    Foot_Force[FR] = low_state.foot_force()[FL];
    Foot_Force[RL] = low_state.foot_force()[RR];
    Foot_Force[RR] = low_state.foot_force()[RL];

    for (int leg = 0; leg < 4; leg++)
    {
        if (Foot_Force[leg] > 13)
        {
            Foot_Contact[leg] = 1;
        }
        else
        {
            Foot_Contact[leg] = 0;
        }
    }
}

void Custom::Joint_Controller()
{
    if (motormode == M_HOMING)
    {
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = 0;
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = 0;
            low_cmd.motor_cmd()[i].kd() = 3;
            low_cmd.motor_cmd()[i].tau() = Torque[i];
            // low_cmd.motor_cmd()[i].tau() = 0;
        }
    }

    else if (motormode == NORMAL)
    {
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = Target_q[i];
            // low_cmd.motor_cmd()[i].q() = 0;  
            low_cmd.motor_cmd()[i].dq() = 0;
            // low_cmd.motor_cmd()[i].kp() = 0;
            low_cmd.motor_cmd()[i].kp() = 22;
            low_cmd.motor_cmd()[i].kd() = 0.7; 
            // low_cmd.motor_cmd()[i].tau() = Torque[i];
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }

    if (Damp_count > 0)
    {
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd()[i].q() = 0;
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = 0;
            low_cmd.motor_cmd()[i].kd() = 5;
            low_cmd.motor_cmd()[i].tau() = 0;
        }

        // std::cout << "DAMPING" << std::endl;
    }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);

    lowcmd_publisher->Write(low_cmd);
}

void Custom::Run()
{
    Motion_Time++;
    Plot_Time += 0.002;



    if (Motion_Time < 500)
    {
        Encoder_Read();
        IMU_Read();
        FootForce_Read();
        GPS_Read();
        High_Observation_Update();
        Observation_Update();
    }
    else if (Motion_Time >= 500)
    {
        Encoder_Read();
        IMU_Read();
        FootForce_Read();
        GPS_Read();
        High_Observation_Update();
        Observation_Update();

        Command(Joint_State);

        if (Motor_State == true)
        {
            Joint_Controller();
        }
    }
}

void Custom::Observation_Update()
{
    ISSAC.Set_IMU(go2_imu_lin_acc, go2_imu_rpy_dot, Quat);
    ISSAC.Set_Encoder(q_, dq_);
    ISSAC.Set_Joystick(x_vel_command, y_vel_command, 0.0, yaw_vel_command, height_command);  
    // ISSAC.Set_Joystick(LPF_High_Command(0), LPF_High_Command(1), 0.0, LPF_High_Command(2), height_command);  
    ISSAC.Set_Observation();
}

void Custom::High_Observation_Update()
{
    HIGH.Set_IMU(go2_imu_lin_acc, go2_imu_rpy_dot, Quat);
    HIGH.Set_Position(Robot_Pos, Target_Pos);
    HIGH.Set_Observation();
}

void Custom::PolicyRun()
{
    if(Policy_time > 200)
    {
        ISSAC.Inference();

        for (int i = 0; i < NUM_LEG; i++)
        { 
            action_leg[i] = ISSAC.GetAction(i);
        }
    }

    Policy_time++;
}

void Custom::HighPolicyRun()
{
    if(Policy_time > 200)
    {
        HIGH.Inference();

        High_Command = HIGH.GetAction();
        LPF_High_Command(0) = LPF(High_Command(0), Pre_High_Command(0), 0.02, 0.01);
        LPF_High_Command(1) = LPF(High_Command(1), Pre_High_Command(1), 0.02, 0.01);
        LPF_High_Command(2) = LPF(High_Command(2), Pre_High_Command(2), 0.02, 0.01);
        Pre_High_Command(0) = High_Command(0);
        Pre_High_Command(1) = High_Command(1);
        Pre_High_Command(2) = High_Command(2);
    }

    Policy_time++;
}

void Custom::JoyRun()
{
    JoyStick_Control();
}

void Custom::FSM_Waypoint()
{
    const double arrive_dist = 1.0;
    double dist = 0.0;

    switch (current_mode)
    {
    case FSM_Mode::start_to_way1:
        dist = distanceToTarget(Robot_Pos, shifted_waypoint[1]);
        if (dist < arrive_dist)
        {
            current_mode = FSM_Mode::way1_to_way2;
            Current_Target = shifted_waypoint[2];
            Target_Pos(0) = Current_Target.x;
            Target_Pos(1) = Current_Target.y;
            std::cout << "way1_to_way2" << std::endl;
        }
        // std::cout << "start_to_way1" << std::endl;
        break;

    case FSM_Mode::way1_to_way2:
        dist = distanceToTarget(Robot_Pos, waypoint[2]);
        if (dist < arrive_dist)
        {
            current_mode = FSM_Mode::way2_to_way3;
            Current_Target = shifted_waypoint[3];
            Target_Pos(0) = Current_Target.x;
            Target_Pos(1) = Current_Target.y;
            std::cout << "way2_to_way3" << std::endl;
        }
            
        // std::cout << "way1_to_way2" << std::endl;
        break;

    case FSM_Mode::way2_to_way3:
        dist = distanceToTarget(Robot_Pos, shifted_waypoint[3]);
        if (dist < arrive_dist)
        {
            current_mode = FSM_Mode::way3_to_way4;
            Current_Target = shifted_waypoint[4];
            Target_Pos(0) = Current_Target.x;
            Target_Pos(1) = Current_Target.y;
            std::cout << "way3_to_way4" << std::endl;
        }
            
        // std::cout << "way2_to_way3" << std::endl;
        break;

    case FSM_Mode::way3_to_way4:
        dist = distanceToTarget(Robot_Pos, shifted_waypoint[4]);
        if (dist < arrive_dist)
        {
            current_mode = FSM_Mode::way4_to_way5;
            Current_Target = shifted_waypoint[5];
            Target_Pos(0) = Current_Target.x;
            Target_Pos(1) = Current_Target.y;
            std::cout << "way4_to_way5" << std::endl;
        }
        // std::cout << "way3_to_way4" << std::endl;
        break;

    case FSM_Mode::way4_to_way5:
        dist = distanceToTarget(Robot_Pos, shifted_waypoint[5]);
        if (dist < arrive_dist)
        {
            current_mode = FSM_Mode::way5_to_start;
            Current_Target = shifted_waypoint[0];
            Target_Pos(0) = Current_Target.x;
            Target_Pos(1) = Current_Target.y;
            std::cout << "way5_to_startS" << std::endl;
            
        }
        // std::cout << "way4_to_way5" << std::endl;
        break;

    case FSM_Mode::way5_to_start:
        dist = distanceToTarget(Robot_Pos, shifted_waypoint[0]);
        if (dist < arrive_dist)
        {
            current_mode = FSM_Mode::start_to_way1;
            Current_Target = shifted_waypoint[1];
            Target_Pos(0) = Current_Target.x;
            Target_Pos(1) = Current_Target.y;
            std::cout << "start_to_way1 " << std::endl;
        }
        // std::cout << "way5_to_start" << std::endl;
        break;
    }

}