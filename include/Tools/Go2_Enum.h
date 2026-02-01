#ifndef GO2_ENUM_H_
#define GO2_ENUM_H_

#define     PI          3.141592
#define     RAD2DEG     180 / PI
#define     DEG2RAD     PI / 180

#define     stance      0
#define     swing       -1

#define     L_0         0.04
#define     L_1         0.215
#define     L_2         0.215

#define     Go2_M           15.0

enum Hertz
{
    HZ_CONTROL = 500,
    HZ_POICY = 50
};

enum Joint
{
    HR,
    HP,
    KP,
    NUM_JOINT = 3,
};

enum Leg_Num
{
    FL,
    FR,
    RL,
    RR,
    NUM_LEG = 4,
};
enum DoF
{
    NUM_DOF = NUM_JOINT * NUM_LEG,
};
enum Pos
{
    X,
    Y,
    Z,
    NUM_AXIS = 3,
};

enum RPY
{
    ROLL,
    PITCH,
    YAW,
    NUM_RPY = 3,
};

enum ControlMode
{
    INIT,
    HOMING,
    WALKING,
    NUM_MODE = 3,
};

enum InitMode
{
    JOINTINIT,
    ENCODERINIT,
    START,
    NUM_INITMODE = 3,
};

enum MotorMode
{
    DAMPING,
    M_HOMING,
    NORMAL,
    NUM_MOTORMODE = 3,
};

enum BodyState
{
    GO2_ROLL,
    GO2_PITCH,
    GO2_YAW,
    GO2_X,
    GO2_Y,
    GO2_Z,
    GO2_ROLL_DOT,
    GO2_PITCH_DOT,
    GO2_YAW_DOT,
    GO2_X_DOT,
    GO2_Y_DOT,
    GO2_Z_DOT,
    GO2_G,
    GO2_NUM = 13,
};

#endif