#include "unitree_arm_sdk/control/unitreeArm.h"
#include "LeapC.h"
#include "leapmotion.hpp"


int main() {

    UNITREE_ARM::unitreeArm arm(false);
    arm.sendRecvThread->start();

    arm.backToStart();

    arm.labelRun("forward");
    arm.startTrack(UNITREE_ARM::ArmFSMState::CARTESIAN);
    
    double angular_vel = 0.3;
    double linear_vel = 0.3;
    UNITREE_ARM::Timer timer(arm._ctrlComp->dt);

    for(int i(0); i<2000;i++){
        arm.directions<< 0, 0, 0, 0, 0, -1, -1;

        arm.cartesianCtrlCmd(arm.directions, angular_vel, linear_vel);
        timer.sleep();
    }
    
    arm.backToStart();

    // float origin[3] = {0, 0, 0};
    // float current[3] = {0, 0, 0};
    // float position[3] = {0, 0, 0};
    // bool isGrabbing = false;
    // bool justGrabbed = false;
    // LEAP_HAND *hand = NULL;
    // OpenConnection();
    // while(!IsConnected)
    //     millisleep(100); //wait a bit to let the connection complete

    // printf("Connected.");
    // LEAP_DEVICE_INFO *deviceProps = GetDeviceProperties();
    // if(deviceProps)
    //     printf("Using device %s.\n", deviceProps->serial);

    // bool running = true;
    // while(running)
    // {
       
    //     LEAP_TRACKING_EVENT *frame = GetFrame();
    //     if(frame)
    //     {
    //         for(uint32_t h = 0; h < frame->nHands; h++)
    //         {
    //             hand = &frame->pHands[h];
    //             if(hand->type == eLeapHandType_Right)
    //             {
    //                 justGrabbed = !isGrabbing && (hand->grab_strength > 0.8);
    //                 isGrabbing = hand->grab_strength > 0.8;
    //                 current[0] = hand->palm.position.x;
    //                 current[1] = hand->palm.position.y;
    //                 current[2] = hand->palm.position.z;
    //             }
    //         }
    //     }

    //     //if hand is not null, draw the hand has a circle using x and y coordinates
    //     if(hand != NULL)
    //     {
    //         if(justGrabbed)
    //         {
    //             origin[0] = current[0] - position[0];
    //             origin[1] = current[1] - position[1];
    //             origin[2] = current[2] - position[2];
    //         }
    //         if(isGrabbing)
    //         {
    //             position[0] = current[0] - origin[0];
    //             position[1] = current[1] - origin[1];
    //             position[2] = current[2] - origin[2];
    //         }
    //         std::cout << "x: " << position[0] << " y: " << position[1] << " z: " << position[2] << std::endl;
    //     }

    // } //ctrl-c to exit



    arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    return 0;
}