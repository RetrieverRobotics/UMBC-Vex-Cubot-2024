/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

#include "api.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "umbc.h"

#include <cstdint>
#include <sys/types.h>

using namespace pros;
using namespace umbc;
using namespace std;

#define MOTOR_RED_GEAR_MULTIPLIER    100
#define MOTOR_GREEN_GEAR_MULTIPLIER  200
#define MOTOR_BLUE_GEAR_MULTIPLIER   600
#define MOTOR_REVERSE                true

// ports for left drive
#define LEFT_FRONT_MOTOR_PORT 16
#define LEFT_MIDDLE_MOTOR_PORT 18
#define LEFT_BACK_MOTOR_PORT  19

// ports for right drive
#define RIGHT_FRONT_MOTOR_PORT 8
#define RIGHT_MIDDLE_MOTOR_PORT 9
#define RIGHT_BACK_MOTOR_PORT  6

// ports for lift
#define LIFT_MOTOR_PORT 5

// ports for intake
#define INTAKE_MOTOR_PORT 12

#define INTAKE_MOVE_SPEED (int)(127/2)

/*
void intake_go_to(bool clockwise, pros::Motor &intake_motor, int encoder_pos = NULL);
struct param {
        bool open;
        pros::Motor *intake;
        int encoder_pos = NULL;
};
Task intake_mover = NULL;    
*/
void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    // initialize left drive
    pros::Motor drive_left_front_motor = pros::Motor(LEFT_FRONT_MOTOR_PORT);
    pros::Motor drive_left_middle_motor = pros::Motor(LEFT_MIDDLE_MOTOR_PORT, MOTOR_REVERSE);
	pros::Motor drive_left_back_motor = pros::Motor(LEFT_BACK_MOTOR_PORT);
    pros::MotorGroup drive_left = 
        pros::MotorGroup(vector<pros::Motor>{drive_left_front_motor, drive_left_middle_motor, drive_left_back_motor});
    drive_left.set_brake_modes(E_MOTOR_BRAKE_COAST);
    drive_left.set_gearing(E_MOTOR_GEAR_GREEN);
	
    // initialize right drive
    pros::Motor drive_right_front_motor = pros::Motor(RIGHT_FRONT_MOTOR_PORT, MOTOR_REVERSE);
    pros::Motor drive_right_middle_motor = pros::Motor(RIGHT_MIDDLE_MOTOR_PORT);
	pros::Motor drive_right_back_motor = pros::Motor(RIGHT_BACK_MOTOR_PORT, MOTOR_REVERSE);
    pros::MotorGroup drive_right =
        pros::MotorGroup(vector<pros::Motor>{drive_right_front_motor, drive_right_middle_motor, drive_right_back_motor});
    drive_right.set_brake_modes(E_MOTOR_BRAKE_COAST);
    drive_right.set_gearing(E_MOTOR_GEAR_GREEN);

    // initialize lift
    pros::Motor lift_motor = pros::Motor(LIFT_MOTOR_PORT);
    pros::MotorGroup lift = pros::MotorGroup(vector<pros::Motor>{lift_motor});
    lift.set_brake_modes(E_MOTOR_BRAKE_HOLD);
    lift.set_gearing(E_MOTOR_GEAR_RED);

    // initialize intake
    int current_pos = 0;
    int target_pos = 0;
    pros::Motor intake_motor(INTAKE_MOTOR_PORT);

    while(1) {

        // set velocity for drive (arcade controls)
        int32_t tank_left_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int32_t tank_right_y = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

        int32_t drive_left_velocity = (int32_t)(((double)(tank_left_y) / (double)E_CONTROLLER_ANALOG_MAX)
                                        * MOTOR_GREEN_GEAR_MULTIPLIER);

        int32_t drive_right_velocity = (int32_t)(((double)(tank_right_y) / (double)E_CONTROLLER_ANALOG_MAX)
                                        * MOTOR_GREEN_GEAR_MULTIPLIER);                                

        drive_left.move_velocity(drive_left_velocity);
        drive_right.move_velocity(drive_right_velocity);

        // set lift position
        if (controller_master->get_digital(E_CONTROLLER_DIGITAL_R1)) {
            lift.move_velocity(MOTOR_RED_GEAR_MULTIPLIER);
        } else if (controller_master->get_digital(E_CONTROLLER_DIGITAL_R2)) {
            lift.move_velocity(-MOTOR_RED_GEAR_MULTIPLIER);
        } else {
            lift_motor.brake();
        }

        /*
        // set intake position (toggle)
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            static bool is_open = false;
            is_open = !is_open;

            if (!intake_mover || (intake_mover && intake_mover->get_state() )) {
                //TO-DO: Change the heap allocation/deletion method to suspending and deleting a previous task

                param *func_param = new param;
                func_param->intake = intake_motor;
                func_param->open = is_open;
                intake_mover = new Task((task_fn_t)intake_go_to, (void*)func_param, "Moving intake");
                delete func_param;
            }
        } */

        // set intake position (manual)
        if (controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake_motor = INTAKE_MOVE_SPEED;
        } else if (controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake_motor = -INTAKE_MOVE_SPEED;
        } else {
            intake_motor = 0;
        }

        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}

/*
void intake_go_to(param *parameters) {
    bool clockwise = parameters->open;
    pros::Motor &intake_motor = *parameters->intake;
    int encoder_pos = parameters->encoder_pos;
    static const int MOTOR_ERROR = 50; // margin of error
    static const int CYCLE_TIME = 500; // in milliseconds

    int start_pos = intake_motor.get_position();
    uint32_t *prev_cycle = new uint32_t(c::millis()); // used to determine if motor is still moving
    
    bool exited_start = false; // check if the motor has actually begun moving
    bool in_range = false;

    intake_motor = clockwise ? 127 : -127;

    while (!in_range) {
        delete prev_cycle;
        prev_cycle = new uint32_t(c::millis()-CYCLE_TIME);
        
        int current_pos = intake_motor.get_position();
        
        exited_start = abs(start_pos - current_pos) > MOTOR_ERROR;
        in_range = !(exited_start && abs(current_pos - intake_motor.get_raw_position(prev_cycle)) < MOTOR_ERROR);
    }

    delete prev_cycle;
    intake_motor = 0;
} */