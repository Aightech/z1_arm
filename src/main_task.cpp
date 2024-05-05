#include "z1_arm.hpp"
#include "LeapC.h"
#include "leapmotion.hpp"
#include <iostream>

#include <joystick.h>

#define NB_ARMS 2

class Joystick : public cJoystick
{
public:
    int selectedArm = 0;
    bool switchReleased = true;
    double speed = 0.01;
};

void connectArms(HandRobot *handRobots, int *udpPorts, int n = NB_ARMS)
{
    for (int i = 0; i < NB_ARMS; i++)
    {
        handRobots[i].name = (i == 0 ? "left" : "right");
        handRobots[i].handType = (i == 0 ? eLeapHandType_Left : eLeapHandType_Right);
        auto ctrlComp = new UNITREE_ARM::CtrlComponents();
        ctrlComp->dt = 0.002; // control period: 500Hz
        ctrlComp->udp = new UNITREE_ARM::UDPPort("127.0.0.1", udpPorts[i], udpPorts[i] + 1, UNITREE_ARM::RECVSTATE_LENGTH, UNITREE_ARM::BlockYN::NO, 500000);
        ctrlComp->armModel = new Z1custom((i == 0 ? false : true)); // no UnitreeGripper
        ctrlComp->armModel->addLoad(0.03);                          // add 0.03kg payload to the end joint
        handRobots[i].arm = new UNITREE_ARM::unitreeArm(ctrlComp);
        handRobots[i].arm->sendRecvThread->start();
        handRobots[i].arm->backToStart();
    }
    for (int i = 0; i < NB_ARMS; i++)
        handRobots[i].arm->labelRun("forward");
}

void tracking_mode(HandRobot *handRobots)
{
    LEAP_TRACKING_EVENT *frame = GetFrame();
    if (frame)
    {
        LEAP_HAND *potentialHand[2] = {NULL, NULL};
        bool handFound[2] = {false, false};
        for (uint32_t h = 0; h < frame->nHands; h++)
        { // for each hand detected
            LEAP_HAND *hand = &frame->pHands[h];
            for (int t = 0; t < NB_ARMS; t++)
            {
                if (hand->type == handRobots[t].handType) // if the hand is left or right
                {
                    if (true)
                    { // if the hand is already being tracked (or no hand has been tracked yet)
                        handRobots[t].updateHand(hand);
                        handFound[t] = true;
                    }
                    else // new hand
                    {
                        potentialHand[t] = hand;
                    }
                }
            }
        }

        if (handFound[0] && handFound[1])
        {
            // test if both hands are close to each other
            Vec3 left = {handRobots[0].current[3], handRobots[0].current[4], handRobots[0].current[5]};
            Vec3 right = {handRobots[1].current[3], handRobots[1].current[4], handRobots[1].current[5]};
            if ((left - right).norm() < 0.1)
            {
                for (int i = 0; i < NB_ARMS; i++)
                {
                    handRobots[i].arm->labelRun("forward");
                    handRobots[i].homePos();
                }
            }
        }

        for (int t = 0; t < NB_ARMS; t++)
        {
            // auto p = arms[t]->lowstate->endPosture.transpose();
            if (!handFound[t] && potentialHand[t] != NULL)
            {
                // use the new hand only if it is not grabbing
                if (potentialHand[t]->grab_strength < handRobots[t].grabStrength_threshold)
                {
                    handRobots[t].hand = potentialHand[t];
                    handRobots[t].updateHand(potentialHand[t]);
                    handFound[t] = true;
                }
            }
            double k[6] = {1, 1, 1, 10, 10, 10};
            double angular_vel = 0;
            double linear_vel = 0.04;
            Vec6 speed = {0, 0, 0, 0, 0, 0}; // coefficients of angular and linear velocity
            if (handFound[t])
            {
                handRobots[t].updatePosition();

                if (handRobots[t].isGrabbing) // if the hand is grabbing, the robot will follow the hand
                {
                    std::cout << handRobots[t].name << " is grabbing ";
                    auto p = handRobots[t].arm->lowstate->endPosture.transpose(); // get the current position of the robot
                    for (int i = 0; i < 6; i++)
                    { // calculate the speed of the robot
                        speed[i] = -k[i] * (p[i] - handRobots[t].position[i]);
                        speed[i] = (speed[i] > 1) ? 1 : ((speed[i] < -1) ? -1 : speed[i]);
                    }
                }
            }
            if (handRobots[t].period())
            {
                handRobots[t].arm->directions << speed[0], speed[1], speed[2], speed[3], speed[4], speed[5], 0;
                handRobots[t].arm->cartesianCtrlCmd(handRobots[t].arm->directions, angular_vel, linear_vel);
            }
        }
        std::cout << "                           \xd" << std::flush;
    }
    else
    {
        for (int i = 0; i < NB_ARMS; i++)
        {
            handRobots[i].arm->directions << 0, 0, 0, 0, 0, 0, 0;
            handRobots[i].arm->cartesianCtrlCmd(handRobots[i].arm->directions, 0, 0);
        }
    }
}

void connectLeap()
{
    OpenConnection();
    while (!IsConnected)
        millisleep(100); // wait a bit to let the connection complete
    LEAP_DEVICE_INFO *deviceProps = GetDeviceProperties();
    if (deviceProps)
        printf("Using device %s.\n", deviceProps->serial);
}

void joystick_mode(Joystick *js, HandRobot *handRobots)
{
    if (js->switchReleased)
    {
        if (abs(js->joystickValue(6)) > 0)
        {
            js->selectedArm = (js->selectedArm + 1) % 2;
            js->switchReleased = false;
        }
    }
    else if (abs(js->joystickValue(6)) == 0)
        js->switchReleased = true;

    js->speed += -js->joystickValue(7) / 32767. / 1000000;
    js->speed = (js->speed < 0.001) ? 0.001 : ((js->speed > 0.04) ? 0.04 : js->speed);
    double angular_vel = 0;
    double linear_vel = js->speed;
    int jIndex[3] = {1, 0, 4};
    for (int i = 0; i < 3; i++)
    {
        double val;
        val = js->joystickValue(jIndex[i]) / 32767.;
        val = (val > 0.05) ? (val - 0.05) : ((val < -0.05) ? val + 0.05 : 0);
        handRobots[js->selectedArm].arm->directions[i + 3] = -val; // reverse direction
    }
    if (handRobots[js->selectedArm].period())
        handRobots[js->selectedArm].arm->cartesianCtrlCmd(handRobots[js->selectedArm].arm->directions, angular_vel, linear_vel);
}

void choree_mode(HandRobot *handRobot)
{
    char n = handRobot->name.c_str()[0];
    std::string name = std::string(1, n);
    handRobot->arm->teachRepeat(name);
}

void teaching_mode(HandRobot *handRobot)
{
    char n = handRobot->name.c_str()[0];
    std::string name = std::string(1, n);
    handRobot->arm->teach(name);
}

typedef enum _runningMode
{
    IDLE,
    PASSIVE,
    HOME,
    FORWARD,
    STOP,
    CHOREE,
    TEACHING,
    TRACKING,
    JOYSTICK
} runningMode;

// This program uses the Leap Motion to get hand position and sends it to the unitree arm to control its position
int main()
{
    HandRobot handRobots[NB_ARMS]; // processing the user hands
    int udpPorts[2] = {8071, 8073};
    connectArms(handRobots, udpPorts, NB_ARMS);
    connectLeap();

    int selectedArm = 0;

    Joystick js;
    try
    {
        js.connect("/dev/input/js0", "");
    }
    catch (std::string err)
    {
        std::cout << err << std::endl;
    }

    bool running = true;
    bool teaching = false;
    runningMode mode = IDLE;
    while (running)
    {
        if (js.buttonPressed(0))
        {
            std::cout << "PASSIVE mode" << std::endl;
        }
        if (js.buttonPressed(1))
        {
            std::cout << "HOME mode" << std::endl;
            mode = HOME;
        }
        if (js.buttonPressed(2))
        {
            std::cout << "FORWARD mode" << std::endl;
            mode = FORWARD;
        }
        if (js.buttonPressed(3))
        {
            if (mode != CHOREE)
                std::cout << "CHOREE mode" << std::endl;
            mode = CHOREE;
        }
        if (js.buttonPressed(4))
        {
            std::cout << "TRACKING mode" << std::endl;
            for (int i = 0; i < NB_ARMS; i++)
            {
                handRobots[i].arm->labelRun("forward");
                handRobots[i].homePos();
            }
            mode = TRACKING;
        }
        if (js.buttonPressed(5))
        {
            if (mode != JOYSTICK)
                std::cout << "JOYSTICK mode" << std::endl;
            mode = JOYSTICK;
        }
        if (js.buttonPressed(6))
        {
            std::cout << "STOP" << std::endl;
            mode = STOP;
        }
        if (js.switchReleased)
        {
            if (js.buttonPressed(8))
            {
                teaching = !teaching;
                if (teaching)
                    std::cout << "TEACHING" << std::endl;
                else
                    std::cout << "STOP TEACHING" << std::endl;
                js.switchReleased = false;
                mode = TEACHING;
            }
        }
        else if (!js.buttonPressed(8))
            js.switchReleased = true;

        switch (mode)
        {
        case IDLE:
            break;
        case PASSIVE:
            for (int i = 0; i < NB_ARMS; i++)
                handRobots[i].arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
            mode = IDLE;
            break;
        case FORWARD:
            for (int i = 0; i < NB_ARMS; i++)
                handRobots[i].arm->labelRun("forward");
            mode = IDLE;
            break;
        case HOME:
            for (int i = 0; i < NB_ARMS; i++)
                handRobots[i].arm->backToStart();
            mode = IDLE;
            break;
        case STOP:
            running = false;
            break;
        case CHOREE:
            std::thread *t[NB_ARMS];
            for (int i = 0; i < NB_ARMS; i++)
            {
                t[i] = new std::thread(choree_mode, &handRobots[i]);
            }
            for (int i = 0; i < NB_ARMS; i++)
            {
                t[i]->join();
                delete t[i];
            }
            mode = IDLE;
            std::cout << "Finished" << std::endl;

            break;
        case TEACHING:
            if (teaching)
            {
                for (int i = 0; i < NB_ARMS; i++)
                    teaching_mode(&handRobots[i]);
            }
            else
            {
                for (int i = 0; i < NB_ARMS; i++)
                    handRobots[i].arm->setFsm(UNITREE_ARM::ArmFSMState::JOINTCTRL);
            }
            mode = IDLE;
            break;
        case TRACKING:
            tracking_mode(handRobots);
            break;
        case JOYSTICK:
            joystick_mode(&js, handRobots);
            break;
        }
    }

    for (int i = 0; i < NB_ARMS; i++)
    {
        handRobots[i].arm->backToStart();
        handRobots[i].arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        handRobots[i].arm->sendRecvThread->shutdown();
    }
    return 0;
}