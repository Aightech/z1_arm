#include "unitree_arm_sdk/control/unitreeArm.h"
#include "LeapC.h"
#include "leapmotion.hpp"
#include <iostream>

int main()
{

    UNITREE_ARM::unitreeArm arm(false);
    arm.sendRecvThread->start();

    arm.backToStart();

    Vec6 posture[2];
    float gripper_pos = 0.0;

    std::cout << "[TO STATE]" << std::endl;
    arm.labelRun("forward");

    // std::cout << "[MOVEJ]" << std::endl;
    // posture[0]<<0,0,0, 0.5,0,0;
    // joint_speed = 2.0;
    // MoveJ(posture[0], gripper_pos, joint_speed);

    // print the current cartesian position
    std::cout << "[GET STATE]" << std::endl;

    float origin[3] = {0, 0, 0};
    float current[3] = {0, 0, 0};
    float position[3] = {0.374, 0.000, 0.406};
    bool isGrabbing = false;
    bool justGrabbed = false;
    LEAP_HAND *hand = NULL;
    int64_t lastFrameID = 0; // The last frame received

    OpenConnection();
    while (!IsConnected)
        millisleep(100); // wait a bit to let the connection complete

    printf("Connected.");
    LEAP_DEVICE_INFO *deviceProps = GetDeviceProperties();
    if (deviceProps)
        printf("Using device %s.\n", deviceProps->serial);

    bool running = true;
    while (running)
    {

        LEAP_TRACKING_EVENT *frame = GetFrame();
        if (frame && frame->nHands > 0)
        {
            for (uint32_t h = 0; h < frame->nHands; h++)
            {
                hand = &frame->pHands[h];
                if (hand->type == eLeapHandType_Left)
                {
                    justGrabbed = !isGrabbing && (hand->grab_strength > 0.8);
                    isGrabbing = hand->grab_strength > 0.8;
                    current[0] = hand->palm.position.z / 1000;
                    current[1] = hand->palm.position.x / 1000;
                    current[2] = hand->palm.position.y / 1000;
                }
            }

            // if hand is not null, draw the hand has a circle using x and y coordinates
            if (hand != NULL)
            {
                if (justGrabbed)
                {
                    origin[0] = current[0] - position[0];
                    origin[1] = current[1] - position[1];
                    origin[2] = current[2] - position[2];
                }
                if (isGrabbing)
                {
                    position[0] = current[0] - origin[0];
                    position[1] = current[1] - origin[1];
                    position[2] = current[2] - origin[2];
                }

                double angular_vel = 0.3;
                double linear_vel = 0.3;
                double k = 0.5;
                float speed[3];
                auto p = arm.lowstate->endPosture.transpose();
                auto t = arm.lowstate->getTau().transpose();
                for (int i = 0; i < 6; i++)
                {
                    if (abs(t[i]) > 10)
                    {
                        isGrabbing = false;
                        std::cout << i << " " << t[i] << std::endl;
                        arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
                    }
                }
                for (int i = 0; i < 3; i++)
                {
                    speed[i] = -k * (p[3 + i] - position[i]);
                }
                if (isGrabbing)
                    arm.directions << 0, 0, 0, speed[0], speed[1], speed[2], 0;
                else
                {
                    
                    arm.directions << 0, 0, 0, 0, 0, 0, 0;
                }
                arm.cartesianCtrlCmd(arm.directions, angular_vel, linear_vel);
            }
        }
        else
        {
            arm.directions << 0, 0, 0, 0, 0, 0, 0;
            arm.cartesianCtrlCmd(arm.directions, 0, 0);
        }
    } // ctrl-c to exit

    arm.backToStart();

    arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    return 0;
}