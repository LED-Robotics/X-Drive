#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::v5::MotorGroup leftFront({-1, 2}, pros::MotorGearset::green, pros::MotorUnits::degrees);
pros::v5::MotorGroup leftBack({-11, 12}, pros::MotorGearset::green, pros::MotorUnits::degrees);
pros::v5::MotorGroup rightFront({9, -10}, pros::MotorGearset::green, pros::MotorUnits::degrees);
pros::v5::MotorGroup rightBack({19, -20}, pros::MotorGearset::green, pros::MotorUnits::degrees);
pros::v5::Motor intakeTop(-4, pros::MotorGearset::green, pros::MotorUnits::degrees);
pros::v5::MotorGroup intakeBottom({3,-5}, pros::MotorGearset::green, pros::MotorUnits::degrees);

pros::v5::Imu gyro(6);

const int normalSpeed = 90;
const int turboSpeed = 127;

void setDriveBrakes(bool state) {
	leftFront.set_brake_mode(state ? MOTOR_BRAKE_BRAKE : MOTOR_BRAKE_COAST);
	leftBack.set_brake_mode(state ? MOTOR_BRAKE_BRAKE : MOTOR_BRAKE_COAST);
	rightFront.set_brake_mode(state ? MOTOR_BRAKE_BRAKE : MOTOR_BRAKE_COAST);
	rightBack.set_brake_mode(state ? MOTOR_BRAKE_BRAKE : MOTOR_BRAKE_COAST);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "bababooey!");
	pros::lcd::set_text(2, "bababooey!");
	pros::lcd::set_text(3, "bababooey!");
	pros::lcd::set_text(4, "bababooey!");
	setDriveBrakes(false);
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	bool brakeState = false;
	bool fieldOriented = false;
	while (true) {
		if(master.get_digital_new_press(DIGITAL_A)) {
			brakeState = !brakeState;
			setDriveBrakes(brakeState);
			master.clear_line(0);
		}
		master.set_text(0, 0, brakeState ? "Brakes: Enabled" : "Brakes: Disabled");


		bool l1 = master.get_digital(DIGITAL_L1);
		bool l2 = master.get_digital(DIGITAL_L2);
		bool r1 = master.get_digital(DIGITAL_R1);
		bool r2 = master.get_digital(DIGITAL_R2);

		if(l1) {
			intakeBottom.move(l1 * (l2 ? turboSpeed : normalSpeed));
			intakeTop.move(l1 * -(l2 ? turboSpeed : normalSpeed));
		} else {
			intakeBottom.move((r1 - r2) * (l2 ? turboSpeed : normalSpeed));
			intakeTop.move((r1 - r2) * (l2 ? turboSpeed : normalSpeed));
		}

		int rot = master.get_analog(ANALOG_RIGHT_X) * 0.8;
		double origX = master.get_analog(ANALOG_LEFT_X);
		double origY = master.get_analog(ANALOG_LEFT_Y);

		if(master.get_digital_new_press(DIGITAL_X)) {
			fieldOriented = !fieldOriented;
			master.clear_line(1);
		}
		master.set_text(2, 0, fieldOriented ? "FOC: Enabled" : "FOC: Disabled");

		double heading, fieldX, fieldY;
		heading = 0.0;

		if(fieldOriented) {
			heading = gyro.get_heading();
		}
		fieldX = cos(heading) * origX - sin(heading) * origY;
		fieldY = sin(heading) * origX - cos(heading) * origY;
		
		int x = (int)fieldX;
		int y = (int)fieldY;

		leftFront.move(y + x + rot);
		leftBack.move(y - x + rot);

		rightFront.move(y - x - rot);
		rightBack.move(y + x - rot);

		pros::delay(20);
	}
}