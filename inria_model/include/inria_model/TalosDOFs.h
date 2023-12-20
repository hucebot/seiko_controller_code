#ifndef INRIA_MODEL_TALOSDOFS_H
#define INRIA_MODEL_TALOSDOFS_H

#include <string>
#include <vector>
#include <map>

namespace inria {

static const std::map<std::string, double> PostureDefault = {
    {"leg_left_1_joint", 0.0},
    {"leg_left_2_joint", 0.0},
    {"leg_left_3_joint", -0.317392},
    {"leg_left_4_joint", 0.610042},
    {"leg_left_5_joint", -0.29265},
    {"leg_left_6_joint", 0.0},
    {"leg_right_1_joint", 0.0},
    {"leg_right_2_joint", 0.0},
    {"leg_right_3_joint", -0.317392},
    {"leg_right_4_joint", 0.610042},
    {"leg_right_5_joint", -0.29265},
    {"leg_right_6_joint", 0.0},
    {"torso_1_joint", 0.0},
    {"torso_2_joint", 0.0},
    {"arm_left_1_joint", 0.218615},
    {"arm_left_2_joint", 0.566335},
    {"arm_left_3_joint", -0.582134},
    {"arm_left_4_joint", -1.4169},
    {"arm_left_5_joint", 0.315339},
    {"arm_left_6_joint", 0.0615923},
    {"arm_left_7_joint", -0.146577},
    {"gripper_left_inner_double_joint", -2e-1},
    {"gripper_left_fingertip_1_joint", 2e-1},
    {"gripper_left_fingertip_2_joint", 2e-1},
    {"gripper_left_inner_single_joint", 2e-1},
    {"gripper_left_fingertip_3_joint", 2e-1},
    {"gripper_left_joint", -0.05},
    {"gripper_left_motor_single_joint", 2e-1},
    {"arm_right_1_joint", -0.218505},
    {"arm_right_2_joint", -0.566321},
    {"arm_right_3_joint", 0.582227},
    {"arm_right_4_joint", -1.41689},
    {"arm_right_5_joint", -0.315343},
    {"arm_right_6_joint", -0.0617214},
    {"arm_right_7_joint", -0.146588},
    {"gripper_right_inner_double_joint", -2e-1},
    {"gripper_right_fingertip_1_joint", 2e-1},
    {"gripper_right_fingertip_2_joint", 2e-1},
    {"gripper_right_inner_single_joint", 2e-1},
    {"gripper_right_fingertip_3_joint", 2e-1},
    {"gripper_right_joint", -0.05},
    {"gripper_right_motor_single_joint", 2e-1},
    {"head_1_joint", 0.0},
    {"head_2_joint", 0.0},

};

static const std::map<std::string, double> StiffnessJoint = {
    {"head_1_joint", 300.0},
    {"head_2_joint", 300.0},
    {"torso_1_joint", 10000.0},
    {"torso_2_joint", 10000.0},
    {"arm_left_1_joint", 10000.0},
    {"arm_left_2_joint", 10000.0},
    {"arm_left_3_joint", 5000.0},
    {"arm_left_4_joint", 10000.0},
    {"arm_left_5_joint", 3000.0},
    {"arm_left_6_joint", 3000.0},
    {"arm_left_7_joint", 3000.0},
    {"arm_right_1_joint", 10000.0},
    {"arm_right_2_joint", 10000.0},
    {"arm_right_3_joint", 5000.0},
    {"arm_right_4_joint", 10000.0},
    {"arm_right_5_joint", 3000.0},
    {"arm_right_6_joint", 3000.0},
    {"arm_right_7_joint", 3000.0},
    {"leg_left_1_joint", 5000.0},
    {"leg_left_2_joint", 5000.0},
    {"leg_left_3_joint", 5000.0},
    {"leg_left_4_joint", 5000.0},
    {"leg_left_5_joint", 5000.0},
    {"leg_left_6_joint", 5000.0},
    {"leg_right_1_joint", 5000.0},
    {"leg_right_2_joint", 5000.0},
    {"leg_right_3_joint", 5000.0},
    {"leg_right_4_joint", 5000.0},
    {"leg_right_5_joint", 5000.0},
    {"leg_right_6_joint", 5000.0},
    {"gripper_left_joint", 1000.0},
    {"gripper_left_inner_double_joint", 20.0},
    {"gripper_left_fingertip_1_joint", 20.0},
    {"gripper_left_fingertip_2_joint", 20.0},
    {"gripper_left_motor_single_joint", 20.0},
    {"gripper_left_fingertip_3_joint", 20.0},
    {"gripper_left_inner_single_joint", 20.0},
    {"gripper_right_joint", 1000.0},
    {"gripper_right_inner_double_joint", 20.0},
    {"gripper_right_fingertip_1_joint", 20.0},
    {"gripper_right_fingertip_2_joint", 20.0},
    {"gripper_right_motor_single_joint", 20.0},
    {"gripper_right_fingertip_3_joint", 20.0},
    {"gripper_right_inner_single_joint", 20.0},
};

static const double FootHalfSizeX = 0.20/2.0;
static const double FootHalfSizeY = 0.13/2.0;

}

#endif

