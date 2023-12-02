#include "main.h"
#include "lemlib/api.hpp"
#include <string>

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor motor_lf(4, pros::E_MOTOR_GEARSET_06, true);
pros::Motor motor_lb(5, pros::E_MOTOR_GEARSET_06, true);
pros::Motor motor_lt(6, pros::E_MOTOR_GEARSET_06, false);
pros::Motor motor_rf(7, pros::E_MOTOR_GEARSET_06, true);
pros::Motor motor_rb(8, pros::E_MOTOR_GEARSET_06, true);
pros::Motor motor_rt(9, pros::E_MOTOR_GEARSET_06, false);
pros::Motor intake(10);
pros::Motor flywheel(13);
pros::Motor_Group leftDrive({motor_lf, motor_lb, motor_lt});
pros::Motor_Group rightDrive({motor_rf, motor_rb, motor_rt});
pros::ADIDigitalOut lift('A');
pros::ADIDigitalOut hang('B');
pros::ADIDigitalOut wings('C');
pros::Imu iner(15);

lemlib::Drivetrain_t drivetrain{
		&leftDrive,	 // left drivetrain motors
		&rightDrive, // right drivetrain motors
		10,					 // track width
		3.25,				 // wheel diameter
		400					 // wheel rpm
};

lemlib::OdomSensors_t sensors{
		nullptr, // vertical tracking wheel 2
		nullptr,
		nullptr, // horizontal tracking wheel 1
		nullptr, // we don't have a second tracking wheel, so we set it to nullptr
		&iner		 // inertial sensor
};

lemlib::ChassisController_t lateralController{
		0.8, // kP
		3,	 // kD
		1,	 // smallErrorRange
		100, // smallErrorTimeout
		3,	 // largeErrorRange
		500, // largeErrorTimeout
		5		 // slew rate
};

lemlib::ChassisController_t angularController{
		0.8, // kP
		3,	 // kD
		1,	 // smallErrorRange
		100, // smallErrorTimeout
		3,	 // largeErrorRange
		500, // largeErrorTimeout
		0		 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

std::string to_string(float number)
{
	return std::to_string(number);
}

void screen()
{
	// loop forever
	while (true)
	{
		pros::Controller master(pros::E_CONTROLLER_MASTER);
		lemlib::Pose pose = chassis.getPose();													// get the current position of the robot
		controller.set_text(0, 0, "x: " + to_string(pose.x));						// print the x position
		controller.set_text(1, 0, "y: " + to_string(pose.y));						// print the y position
		controller.set_text(2, 0, "heading: " + to_string(pose.theta)); // print the heading
		pros::delay(70);																								// delay for 20 milliseconds
	}
}

// Runs initialization code. This occurs as soon as the program is started.
// All other competition modes are blocked by initialize; it is recommended to keep execution time for this mode under a few seconds.
void initialize()
{
	pros::lcd::initialize();
	chassis.calibrate();
	chassis.setPose(0, 0, 0);
	controller.clear();
}

// Runs when robot is in disabled state of Field Management System or VEX Competition Switch, following either autonomous or opcontrol. When the robot is enabled, this task will exit.
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
void autonomous()
{
	pros::Task screenTask(screen);
	chassis.turnTo(30, 0, 2000);
}

/**
 * Runs operator control code. Will be started in its own task
 * with default priority and stack size when robot is enabled via
 * Field Management System or VEX Competition Switch in operator
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
	float power, turn;
	bool buttonR1Pressed, buttonR2Pressed, buttonYPressed, buttonXPressed, buttonRightPressed;
	bool flywheelOn, flywheelPressed = false;
	bool toggleLift, liftPressed = false;
	bool toggleHang, hangPressed = false;

	motor_lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	motor_lf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	motor_lt.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	motor_rb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	motor_rf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	motor_rt.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	flywheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	controller.set_text(0, 0, "Example");
	pros::delay(100);
	controller.clear();

	while (true)
	{
		pros::Task screenTask(screen);
		// Get joystick values
		power = controller.get_analog(ANALOG_LEFT_X);
		turn = controller.get_analog(ANALOG_LEFT_Y);

		// Check if buttons are pressed
		buttonR1Pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		buttonR2Pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		buttonYPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
		buttonXPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
		buttonRightPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);

		// Set motor velocities based on joystick inputs and button state
		leftDrive.move(6 * (power + turn) / 1.27);
		rightDrive.move(6 * (power - turn) / 1.27);

		// Set intake motor velocity based on button state
		if (buttonR1Pressed == 1)
		{
			intake.move(600);
		}
		else if (buttonR2Pressed == 1)
		{
			intake.move(-600);
		}
		else
		{
			intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			intake.brake();
		}

		// Toggle flywheel based on button state
		if (flywheelOn)
		{
			flywheel.move(600);
		}
		else
		{
			flywheel.brake();
		}

		if (buttonYPressed)
		{
			if (!flywheelPressed)
			{
				flywheelOn = !flywheelOn;
				flywheelPressed = true;
			}
		}
		else
		{
			flywheelPressed = false;
		}

		// Enable hang if arrow key is pressed
		if (buttonRightPressed)
		{
			if (!hangPressed)
			{
				toggleHang = !toggleHang;
				hangPressed = true;
			}
		}
		else
		{
			hangPressed = false;
		}

		if (toggleHang)
		{
			hang.set_value(1);
		}
		else
		{
			hang.set_value(0);
		}

		// Toggle lift based on button state
		if (buttonXPressed)
		{
			if (!liftPressed)
			{
				toggleLift = !toggleLift;
				liftPressed = true;
			}
		}
		else
		{
			liftPressed = false;
		}

		if (toggleLift)
		{
			lift.set_value(1);
		}
		else
		{
			lift.set_value(0);
		}

		pros::delay(20); // Typical delay in an opcontrol loop
	}
}