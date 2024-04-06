#include "unitree_arm_sdk/control/unitreeArm.h"

using namespace UNITREE_ARM;


int main() {
    std::cout << std::fixed << std::setprecision(3);
    unitreeArm arm(false);
    arm.sendRecvThread->start();

    arm.backToStart();

    arm.labelRun("forward");
    arm.startTrack(ArmFSMState::CARTESIAN);
    
    double angular_vel = 0.3;
    double linear_vel = 0.3;
    Timer timer(arm._ctrlComp->dt);

    for(int i(0); i<2000;i++){
        arm.directions<< 0, 0, 0, 0, 0, -1, -1;

        arm.cartesianCtrlCmd(arm.directions, angular_vel, linear_vel);
        timer.sleep();
    }
    
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    return 0;
}