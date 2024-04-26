#include "unitree_arm_sdk/control/unitreeArm.h"
#include "LeapC.h"
#include "leapmotion.hpp"
#include <iostream>

#define NB_ARMS 2

struct NoGOZone
{
    NoGOZone(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
    {
        this->x_min = x_min;
        this->x_max = x_max;
        this->y_min = y_min;
        this->y_max = y_max;
        this->z_min = z_min;
        this->z_max = z_max;
    }
    NoGOZone(float x0, float y0, float z0, float width, float height = -1, float depth = -1)
    {
        this->x_min = x0 - width / 2;
        this->x_max = x0 + width / 2;
        this->y_min = y0 - (height == -1 ? width : height) / 2;
        this->y_max = y0 + (height == -1 ? width : height) / 2;
        this->z_min = z0 - (depth == -1 ? width : depth) / 2;
        this->z_max = z0 + (depth == -1 ? width : depth) / 2;
    }
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
};

struct HandRobot
{
public:
    Vec6 origin = {0, 0, 0, 0, 0, 0};               // origin of the hand movement
    Vec6 current = {0, 0, 0, 0, 0, 0};              // current position of the hand
    Vec6 position = {0, 0, 0, 0.374, 0.000, 0.406}; // position of the robot
    Vec6 current_q;                                 // joint angles of the robot
    Vec3 robotBase = {0, 0, 0};                     // position of the robot base
    bool isGrabbing = false;
    bool justGrabbed = false;
    float grabStrength = 0.8;
    LEAP_HAND *hand = NULL;
    std::vector<NoGOZone> noGoZones;

    void updateHand(LEAP_HAND *hand)
    {
        if (hand->type == eLeapHandType_Left)
        {
            justGrabbed = !isGrabbing && (hand->grab_strength > grabStrength);
            isGrabbing = hand->grab_strength > grabStrength;
            current[0] = hand->palm.orientation.x;
            current[1] = hand->palm.orientation.y;
            current[2] = hand->palm.orientation.z;
            current[3] = hand->palm.position.z / 1000; // convert to meters
            current[4] = hand->palm.position.x / 1000;
            current[5] = hand->palm.position.y / 1000;
        }
    };

    void updatePosition()
    {
        if (justGrabbed)
            for (int i = 0; i < 6; i++)
                origin[i] = current[i] - position[i]; // update the origin of the hand movement
        if (isGrabbing)
        {
            Vec6 newpos = {current[0] - origin[0], current[1] - origin[1], current[2] - origin[2], current[3] - origin[3], current[4] - origin[4], current[5] - origin[5]};
            if (!willCollide(newpos, true))
                for (int i = 0; i < 6; i++)
                    position[i] = current[i] - origin[i]; // update the position of the robot
            else
                std::cout << "Collision detected" << std::endl;
        }
    };

    bool willCollide(Vec6 newpos, bool robotBody = false)
    {
        // check if the endeffector will collide with any of the no-go zones
        for (auto &zone : noGoZones)
        {
            if (newpos[0] + robotBase[0] > zone.x_min && newpos[0] + robotBase[0] < zone.x_max && newpos[1] + robotBase[1] > zone.y_min && newpos[1] + robotBase[1] < zone.y_max && newpos[2] + robotBase[2] > zone.z_min && newpos[2] + robotBase[2] < zone.z_max) // if the next position is in the no-go zone
                return true;
        }
        if (robotBody)
        {
            // check if the robot body will collide with any of the no-go zones
            // compute the new q values of the robot
            Vec6 new_q = current_q;
            // compute the homogenous matrix of the new position
            Mat3 R = robo::rpyToRotMat(newpos[0], newpos[1], newpos[2]);
            Vec3 p = {newpos[3], newpos[4], newpos[5]};
            HomoMat T = robo::RpToTrans(R, p);
            bool success = UNITREE_ARM::Z1Model().inverseKinematics(T, current_q, new_q, true);
            if (success)
            {
                // modelise the robot bodies as 6 segments
                // get the position of each joint and compute if the segment between two joints will collide with the no-go zone
                Vec3 prevPos = UNITREE_ARM::Z1Model().forwardKinematics(current_q, 0).block(0, 3, 3, 1) + robotBase;
                for (int i = 1; i < 7; i++)
                {
                    Vec3 nextPos = UNITREE_ARM::Z1Model().forwardKinematics(new_q, i).block(0, 3, 3, 1) + robotBase;
                    for (auto &zone : noGoZones)
                    {
                        if (nextPos[0] > zone.x_min && nextPos[0] < zone.x_max && nextPos[1] > zone.y_min && nextPos[1] < zone.y_max && nextPos[2] > zone.z_min && nextPos[2] < zone.z_max) // if the next position is in the no-go zone
                            return true;

                        // check if the segment between the two joints will collide with the no-go zone
                        Vec3 dir = nextPos - prevPos;
                        float length = dir.norm();
                        dir.normalize();
                        for (int j = 0; j < 6; j++)
                        {
                            Vec3 pos = prevPos + dir * (length * j / 6);
                            if (pos[0] > zone.x_min && pos[0] < zone.x_max && pos[1] > zone.y_min && pos[1] < zone.y_max && pos[2] > zone.z_min && pos[2] < zone.z_max)
                                return true;
                        }
                    }
                    prevPos = nextPos;
                }
                return false;
            }
            else
                return true; // if the inverse kinematics fails, there might some issues with the robot configuration
        }
    };
};

// This program uses the Leap Motion to get hand position and sends it to the unitree arm to control its position
int main()
{
    UNITREE_ARM::unitreeArm *arms[NB_ARMS];
    int udpPorts[2] = {8071, 8073};
    for (int i = 0; i < NB_ARMS; i++)
    {
        auto ctrlComp = new UNITREE_ARM::CtrlComponents();
        ctrlComp->dt = 0.002; // control period: 500Hz
        ctrlComp->udp = new UNITREE_ARM::UDPPort("127.0.0.1", udpPorts[i], udpPorts[i] + 1, UNITREE_ARM::RECVSTATE_LENGTH, UNITREE_ARM::BlockYN::NO, 500000);
        ctrlComp->armModel = new UNITREE_ARM::Z1Model(); // no UnitreeGripper
        ctrlComp->armModel->addLoad(0.03);               // add 0.03kg payload to the end joint
        arms[i] = new UNITREE_ARM::unitreeArm(ctrlComp);
        arms[i]->sendRecvThread->start();
        arms[i]->backToStart();
        arms[i]->labelRun("forward");
    }

    OpenConnection();
    while (!IsConnected)
        millisleep(100); // wait a bit to let the connection complete
    LEAP_DEVICE_INFO *deviceProps = GetDeviceProperties();
    if (deviceProps)
        printf("Using device %s.\n", deviceProps->serial);

    HandRobot handRobot[NB_ARMS]; // processing the user hands
    // the two robots are put on the user shoulders X_A and X_B and the origin X_O is the center of the user neck
    //...^.Z.......___________.............
    //...|........|...........|............
    //.Y.o-->.X...|..O.....O..|............
    //............|.....<.....|............
    //............|..;____;...|............
    //....X_A.....|___________|.....X_B....
    //....[O].........|[O]|.........[O]....
    //../ooooooooooooo.X_O.oooooooooooo\...
    //..|OOOOOOOOOOOOOOOOOOOOOOOOOOOOOO|...
    //..|OOOO|.|OOOOOOOOOOOOOOOO|.|OOOO|...
    //..|OOOO|.|OOOOOOOOOOOOOOOO|.|OOOO|...
    //.....................................

    handRobot[0].robotBase = {0.3, -0.05, 0};  // robot A is on the left shoulder
    handRobot[1].robotBase = {-0.3, -0.05, 0}; // robot B is on the right shoulder
    NoGOZone noGoZoneBody(0, 0, -0.25, 0.75, 0.4, 0.5);
    NoGOZone noGoZoneHead(0, 0, 0.2, 0.75, 0.4, 0.4);
    for (int i = 0; i < NB_ARMS; i++)
    {
        handRobot[i].noGoZones.push_back(noGoZoneBody);
        handRobot[i].noGoZones.push_back(noGoZoneHead);
    }

    int64_t lastFrameID = 0; // The last frame received
    eLeapHandType handTypes[2] = {eLeapHandType_Left, eLeapHandType_Right};
    bool running = true;
    while (running)
    {
        LEAP_TRACKING_EVENT *frame = GetFrame();
        if (frame && frame->tracking_frame_id > lastFrameID)
        {
            lastFrameID = frame->tracking_frame_id;
            LEAP_HAND *potentialHand[2] = {NULL, NULL};
            bool handFound[2] = {false, false};
            for (uint32_t h = 0; h < frame->nHands; h++)
            { // for each hand detected
                LEAP_HAND *hand = &frame->pHands[h];
                for (int t = 0; t < NB_ARMS; t++)
                {
                    if (hand->type == handTypes[t]) // if the hand is left or right
                    {
                        if (handRobot[t].hand == NULL || hand->id == handRobot[t].hand->id)
                        { // if the hand is already being tracked (or no hand has been tracked yet)
                            handRobot[t].updateHand(hand);
                            handFound[t] = true;
                        }
                        else // new hand
                        {
                            potentialHand[t] = hand;
                        }
                    }
                }
            }

            for (int t = 0; t < NB_ARMS; t++)
            {
                if (!handFound[t] && potentialHand[t] != NULL)
                {
                    // use the new hand only if it is not grabbing
                    if (potentialHand[t]->grab_strength < handRobot[t].grabStrength)
                    {
                        handRobot[t].hand = potentialHand[t];
                        handRobot[t].updateHand(potentialHand[t]);
                        handFound[t] = true;
                    }
                }

                //compute the force applied at the end effector
                Vec6 q = Eigen::Map<Vec6>(arms[t]->_ctrlComp->lowstate->q, 6);

                Vec6 FT = arms[t]->_ctrlComp->armModel->inverseDynamics(Ei

                double k = 0.5;
                double angular_vel = 0.3;
                double linear_vel = 0.3;
                float speed[6] = {0, 0, 0, 0, 0, 0}; // coefficients of angular and linear velocity
                if (handFound[t])
                {
                    handRobot[t].updatePosition();
                    if (handRobot[t].isGrabbing)
                    {                                                       // if the hand is grabbing, the robot will follow the hand
                        auto p = arms[t]->lowstate->endPosture.transpose(); // get the current position of the robot
                        for (int i = 0; i < 6; i++)                         // calculate the speed of the robot
                            speed[i] = -k * (p[3 + i] - handRobot[t].position[i]);
                    }
                }
                arms[t]->directions << speed[0], speed[1], speed[2], speed[3], speed[4], speed[5], 0;
                arms[t]->cartesianCtrlCmd(arms[t]->directions, angular_vel, linear_vel);
            }
        }
        else
        {
            for (int i = 0; i < NB_ARMS; i++)
            {
                arms[i]->directions << 0, 0, 0, 0, 0, 0, 0;
                arms[i]->cartesianCtrlCmd(arms[i]->directions, 0, 0);
            }
        }
    } // ctrl-c to exit

    for (int i = 0; i < NB_ARMS; i++)
    {
        arms[i]->backToStart();
        arms[i]->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        arms[i]->sendRecvThread->shutdown();
    }
    return 0;
}