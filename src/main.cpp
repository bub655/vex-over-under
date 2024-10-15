#include "main.h"
#include "lemlib/api.hpp"
#include <string>
#include <bits/stdc++.h>
using namespace pros;
ASSET(bowl_txt);
ASSET(first_txt);
ASSET(second_txt);
ASSET(third_txt);
ASSET(fourth_txt);
ASSET(fifth_txt);
ASSET(sixth_txt);

Controller controller(E_CONTROLLER_MASTER);
Motor motor_lf(4, E_MOTOR_GEARSET_06, true);
Motor motor_lm(5, E_MOTOR_GEARSET_06, false);
Motor motor_lb(6, E_MOTOR_GEARSET_06, false);
Motor motor_rf(7, E_MOTOR_GEARSET_06, true);
Motor motor_rm(8, E_MOTOR_GEARSET_06, true);
Motor motor_rb(9, E_MOTOR_GEARSET_06, false);
Motor left_intake(20, true);
Motor right_intake(13, false);
Motor_Group intake({left_intake, right_intake});
Motor flywheel(11, true);
Motor_Group rightDrive({motor_rf, motor_rm, motor_rb});
Motor_Group leftDrive({motor_lf, motor_lm, motor_lb});
ADIDigitalOut lift('A');
ADIDigitalOut hang('B');
ADIDigitalOut leftwing('G');
ADIDigitalOut rightwing('H');
Imu iner(17);

lemlib::Drivetrain drivetrain{
		&leftDrive,	 // left drivetrain motors
		&rightDrive, // right drivetrain motors
		11.4,				 // track width
		3.25,				 // wheel rpm
		600,				 // gain
		2,					 // chase power
};

lemlib::OdomSensors sensors{
		nullptr, // vertical tracking wheel 2
		nullptr,
		nullptr, // horizontal tracking wheel 1
		nullptr, // we don't have a second tracking wheel, so we set it to nullptr
		&iner		 // inertial sensor
};
/*
kP, kI, kD, windupRange, smallError, smallErrorTimeout,
largeError, largeErrorTimeout, slew
*/

// forward/backward PID
lemlib::ControllerSettings lateralSettings{
		40,		// kP
		0,		// kI
		50,		// kD
		10,		// windupRange
		0.25, // smallErrorRange
		300,	// smallErrorTimeout
		3,		// largeErrorRange
		500,	// largeErrorTimeout
		100		// slew rate
};

// turning PID
lemlib::ControllerSettings angularSettings{
		15,	 // kP
		0,	 // kI
		94,	 // kD
		10,	 // windupRange
		1,	 // smallErrorRange
		300, // smallE rrorTimeout
		3,	 // largeErrorRange
		500, // largeErrorTimeout
		100	 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralSettings, angularSettings, sensors);

void screenTask()
{
	// loop forever
	while (true)
	{
		lemlib::Pose pose = chassis.getPose(); // get the current position of the robot

		// controller.print(0, 0, "x: %f", pose.x); // print the x position
		// delay(50);
		// controller.print(1, 0, "y: %f", pose.y); // print the y position
		// delay(50);
		// controller.print(2, 0, "heading: %f", pose.theta); // print the heading
		// delay(50);
		pros::lcd::clear();
		pros::lcd::print(1, "x: %f in", pose.x);						// print the x position
		pros::lcd::print(2, "y: %f in", pose.y);						// print the y position
		pros::lcd::print(3, "heading: %f deg", pose.theta); // print the heading
		pros::delay(10);

		// lcd::print(0, "x: %f", pose.x);						// print the x position
		// lcd::print(1, "y: %f", pose.y);						// print the y position
		// lcd::print(2, "heading: %f", pose.theta); // print the heading
		// delay(10);
	}
}

// Runs initialization code. This occurs as soon as the program is started.
// All other competition modes are blocked by initialize; it is recommended to keep execution time for this mode under a few seconds.
void initialize()
{
	lcd::initialize();
	motor_rf.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_rm.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_rb.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_lf.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_lm.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_lb.set_brake_mode(E_MOTOR_BRAKE_COAST);
	flywheel.set_brake_mode(E_MOTOR_BRAKE_COAST);
	left_intake.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	right_intake.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	chassis.calibrate();
	chassis.setPose(0, 0, 0);
	new pros::Task{screenTask};
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

void skills()
{
	// intake.set_brake_modes(MOTOR_BRAKE_COAST);
	// chassis.setPose(0, 0, 135);
	// chassis.moveToPose(-9, 18, 180, 1750, {forwards : false});
	// chassis.waitUntilDone();
	// leftDrive.move(-127);
	// rightDrive.move(-127);
	// delay(300);
	// chassis.moveToPose(0, 14, 90, 1500);
	// chassis.moveToPose(-15.5, 11, 75, 2500, {forwards : false});
	// chassis.waitUntilDone();
	// intake.move(-127);
	// delay(28000);
	// intake.brake();
	// chassis.moveToPoint(chassis.getPose().x + 50, chassis.getPose().y + 15, 4000);
	// // leftwing.set_value(1);
	// chassis.waitUntilDone();
	// chassis.turnTo(chassis.getPose().x - 4, -10, 100);
	// chassis.moveToPoint(chassis.getPose().x + 3, 90, 2500, false);
	// chassis.waitUntilDone();
	// leftDrive.move(-127);
	// rightDrive.move(-127);
	// delay(400);
	// leftDrive.move(50);
	// rightDrive.move(50);
	// delay(400);
	// leftDrive.move(-127);
	// rightDrive.move(-127);
	// delay(300);
	// leftDrive.move(50);
	// rightDrive.move(50);
	// delay(200);
	// chassis.moveToPose(6, 115, 0, 3500);
	// chassis.waitUntilDone();
	// // leftwing.set_value(1);
	// chassis.turnTo(chassis.getPose().x + 30, chassis.getPose().y + 9, 1500);
	// chassis.waitUntilDone();
	// intake.tare_position();
	// intake.move_absolute(-1000, 127);
	// delay(2000);
	// intake.set_brake_modes(MOTOR_BRAKE_HOLD);
	// intake.brake();
	// chassis.waitUntilDone();
	// chassis.setPose(-33.651, 58.73, 98.774);
	// chassis.follow(bowl_txt, 12, 3750);
	// chassis.waitUntilDone();
	// intake.set_brake_modes(MOTOR_BRAKE_COAST);
	// leftDrive.move(127);
	// rightDrive.move(127);
	// delay(600);
	// leftDrive.move(-75);
	// rightDrive.move(-75);
	// delay(200);
	// chassis.moveToPose(61.877, 22.252, 86.649, 2500);
	// chassis.setPose(61.877, 22.252, 180);
	// chassis.follow(first_txt, 12, 3000, false);
	// chassis.turnTo(chassis.getPose().x - 20, chassis.getPose().y, 800);
	// chassis.waitUntilDone();
	// leftwing.set_value(1);
	// rightwing.set_value(1);
	// chassis.follow(second_txt, 12, 4000);
	// chassis.waitUntilDone();
	// leftDrive.move(127);
	// rightDrive.move(127);
	// delay(500);
	// leftDrive.move(-50);
	// rightDrive.move(-50);
	// delay(250);
	// leftDrive.move(127);
	// rightDrive.move(127);
	// delay(500);
	// leftDrive.move(0);
	// rightDrive.move(0);
	// leftwing.set_value(0);
	// rightwing.set_value(0);
	// chassis.setPose(49.793, 0.365, 93.242);
	// chassis.follow(third_txt, 12, 4000, false);
	// chassis.waitUntilDone();
	// leftwing.set_value(1);
	// rightwing.set_value(1);
	// chassis.follow(fourth_txt, 12, 4000);
	// chassis.waitUntilDone();
	// leftDrive.move(127);
	// rightDrive.move(127);
	// delay(500);
	// leftDrive.move(0);
	// rightDrive.move(0);
	// leftwing.set_value(0);
	// rightwing.set_value(0);
	// chassis.follow(fifth_txt, 12, 4000, false);
	// chassis.waitUntilDone();
	// chassis.follow(sixth_txt, 12, 4000);
	// chassis.waitUntilDone();
	// leftDrive.move(127);
	// rightDrive.move(127);
	// delay(500);
	// leftDrive.move(-75);
	// rightDrive.move(-75);
	// delay(400);
	// rightwing.set_value(1);
	// leftDrive.move(127);
	// rightDrive.move(127);
	// delay(800);
	// leftDrive.move(-127);
	// rightDrive.move(-127);
	// delay(400);
	// leftDrive.move(0);
	// rightDrive.move(0);
}
void old_skills()
{
	intake.set_brake_modes(MOTOR_BRAKE_COAST);
	intake.move(127);
	chassis.setPose(0, 0, -30);
	chassis.moveToPoint(-15, 32, 1200);
	chassis.waitUntilDone();
	chassis.moveToPose(-17, 10, 80, 1750, {forwards : false});
	chassis.waitUntilDone();
	delay(25000);

	// flywheel.brake();

	chassis.setPose(-58.071, -45.246, 70);
	// intake.move(-127);
	leftwing.set_value(1);
	chassis.follow(bowl_txt, 12, 4250);
	chassis.waitUntilDone();
	intake.move(0);
	leftwing.set_value(0);
	// intake.set_brake_modes(MOTOR_BRAKE_COAST);
	leftDrive.move(127);
	rightDrive.move(127);
	delay(500);
	chassis.moveToPoint(59.438, -30.018, 1000);
	chassis.turnTo(0, -30.018, 800);
	chassis.follow(first_txt, 12, 4000);
	delay(1000);
	leftwing.set_value(1);
	rightwing.set_value(1);
	chassis.waitUntilDone();
	leftDrive.move(127);
	rightDrive.move(127);
	delay(500);
	leftwing.set_value(0);
	rightwing.set_value(0);
	chassis.follow(second_txt, 12, 2000, false);
	chassis.waitUntilDone();
	leftwing.set_value(1);
	rightwing.set_value(1);
	chassis.turnTo(19.49, 35.089, 750);
	chassis.waitUntilDone();
	chassis.follow(third_txt, 12, 2000);
	chassis.waitUntilDone();
	chassis.follow(fourth_txt, 12, 2000);
}

void rush_5ball()
{
	// BALL 1
	intake.move(127);
	chassis.setPose(0, 0, 0);
	chassis.moveToPose(-9, 50, -10, 1200);
	chassis.waitUntilDone();
	chassis.moveToPoint(-8, 44, 500);
	delay(100);
	intake.move(0);
	chassis.waitUntilDone();
	chassis.turnTo(20, 45, 500);
	chassis.waitUntilDone();
	intake.move(-127);
	chassis.moveToPoint(21, 45, 800);
	chassis.moveToPoint(0, 45, 450, false);
	//  BALL 2
	chassis.waitUntilDone();
	intake.move(127);
	chassis.moveToPose(-30, 25, -90, 1750);
	chassis.waitUntilDone();
	delay(300);
	intake.move(0);
	chassis.moveToPose(5, -10, 50, 2000, {forwards : false});
	chassis.waitUntilDone();
	intake.move(-127);
	delay(300);

	// // BALL 3 (under elevation bar)
	chassis.turnTo(-22, -11, 700);
	chassis.waitUntilDone();
	intake.move(127);
	chassis.moveToPoint(-22, -11, 1000);
	chassis.waitUntilDone();
	delay(300);
	intake.move(0);
	chassis.waitUntilDone();

	// // take out matchload ball
	chassis.moveToPoint(8, -10, 1000, false);
	chassis.turnTo(12, -6, 800);
	// chassis.moveToPose(19, 0, 20, 1500);
	// chassis.waitUntilDone();

	// chassis.turnTo(19, 30, 800);
	// chassis.waitUntilDone();
	// rightwing.set_value(true);
	// intake.move(-127);
	// leftDrive.move(120);
	// rightDrive.move(127);
	// delay(800);
	// leftDrive.move(-127);
	// rightDrive.move(-127);
	// delay(400);
	// leftDrive.move(127);
	// rightDrive.move(127);
	// delay(800);
}

void disrupt_wp()
{
	// steal center ball
	intake.move(127);
	chassis.setPose(0, 0, 10);
	chassis.moveToPoint(9, 42, 1400);
	chassis.waitUntilDone();
	// outtake to bowl
	// chassis.moveToPoint(8, 27, 1000, false);
	// delay(100);
	// intake.move(0);
	// chassis.waitUntilDone();
	// chassis.turnTo(2, 0, 800);
	// delay(300);
	// intake.move(-127);
	// chassis.waitUntilDone();
	// leftDrive.move(-50);
	// rightDrive.move(-50);
	// delay(400);
	// get barrier ball
	// chassis.turnTo(29, 30, 800);
	// chassis.waitUntilDone();
	// intake.move(127);
	// chassis.moveToPose(31, 40.5, 30, 1700);
	// chassis.waitUntilDone();

	// delay(200);
	// intake.move(0);

	// go to matchload
	chassis.moveToPose(-23, 9, 90, 2250, {forwards : false});
	delay(300);
	intake.move(0);
	chassis.waitUntilDone();
	chassis.turnTo(-3, -5, 800);
	chassis.waitUntilDone();
	delay(200);
	rightwing.set_value(1);
	chassis.moveToPoint(-3, -5, 1000);
	chassis.waitUntilDone();
	leftDrive.move(-127);
	rightDrive.move(127);
	delay(200);
	leftDrive.move(0);
	rightDrive.move(0);
	chassis.turnTo(0, -6, 600);
	delay(200);
	rightwing.set_value(0);
	chassis.waitUntilDone();
	intake.move(-127);
	chassis.moveToPose(15, -6, 90, 1500);
	chassis.waitUntilDone();
	chassis.moveToPoint(-4.3, -6, 2000, false);
	chassis.waitUntilDone();
	intake.move(0);
}

void near_wp_score()
{
	chassis.setPose(0, 0, 135);
	chassis.moveToPoint(-4, 4, 750, false);
	chassis.waitUntilDone();
	rightwing.set_value(1);
	chassis.waitUntilDone();
	leftDrive.move(-127);
	rightDrive.move(127);
	delay(300);
	rightwing.set_value(0);
	chassis.moveToPose(-10, 20, 180, 1500, {forwards : false});
	chassis.waitUntilDone();
	leftDrive.move(-127);
	rightDrive.move(-127);
	delay(300);
	chassis.moveToPose(5, 0, 110, 1500);
	chassis.moveToPose(38, -9, 90, 2000);
};
void near_wp()
{
	// chassis.setPose(0, 0, 135);
	// chassis.moveToPoint(-8, 8, 1000, false);
	// chassis.waitUntilDone();
	// rightwing.set_value(1);
	// chassis.moveToPoint(3, -3, 800);
	// chassis.turnTo(10, -3, 500);
	// chassis.waitUntilDone();
	// rightwing.set_value(0);
	// chassis.moveToPose(-10, 20, 180, 1500, {forwards : false});
	// chassis.waitUntilDone();
	// leftDrive.move(-127);
	// rightDrive.move(-127);
	// delay(300);
	// chassis.moveToPose(40, -4, 90, 2000);
	chassis.setPose(0, 0, 0);
	leftwing.set_value(1);
	chassis.turnTo(-5, 0, 750);
	chassis.waitUntilDone();
	leftwing.set_value(0);
	chassis.turnTo(10, -6, 1000);
	chassis.waitUntilDone();
	// chassis.turnTo(-5, 2, 300);
	// chassis.moveToPoint(-8, 0, 750);
	// chassis.waitUntilDone();
	// leftwing.set_value(0);
	// chassis.moveToPose(-13.5, 24, 0, 1500);

	// chassis.waitUntilDone();
	// intake.move(-127);
	// leftDrive.move(127);
	// rightDrive.move(127);
	// delay(500);
	// intake.move(0);
	// chassis.moveToPose(4, -4, -90, 1500, {forwards : false});
	// chassis.turnTo(10, -4, 800);
	intake.move(-127);
	chassis.moveToPose(38, -9, 90, 1700);
	chassis.waitUntilDone();
	delay(400);
	intake.move(0);
}

void push_ball()
{
	chassis.setPose(0, 0, 0);
	leftDrive.move(127);
	rightDrive.move(127);
	delay(2000);
	leftDrive.move(0);
	rightDrive.move(0);
}

void one_ball()
{
	chassis.setPose(0, 0, 30);
	// chassis.moveToPose(15, 30, 0, 1500);
	// chassis.waitUntilDone();
	intake.move(127);
	leftDrive.move(127);
	rightDrive.move(127);
	delay(900);
	leftDrive.move(0);
	rightDrive.move(0);
	intake.move(0);
}
void autonomous() // Ashwath was here
{
	skills();
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

	motor_rm.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_rf.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_rb.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_lm.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_lf.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_lb.set_brake_mode(E_MOTOR_BRAKE_COAST);
	float power, turn;
	bool flywheelOn, flywheelPressed = false;
	bool toggleLift, liftPressed = false;
	bool toggleHang, hangPressed = false;
	bool toggleWings, wingsPressed = false;
	bool toggleDrive, drivePressed = false;
	bool toggleIntake = false;
	float x, y;
	bool arcade = true;

	while (true)
	{

		// Task screenTask(screen)
		// Get joystick values
		if (arcade)
		{
			x = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
			y = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
			turn = 0.55 * (x * pow(fabs(x) / (127), 1.4));
			power = y * pow(fabs(y) / (127), 1.2);
			// chassis.arcade(y, x, 2.7);
			// Set motor velocities based on joystick inputs and button state
			leftDrive.move((turn + power));
			rightDrive.move((power - turn));
		}
		else
		{
			x = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
			y = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
			x = (x * pow(fabs(x) / (127), 2));
			y = y * pow(fabs(y) / (127), 2);
			leftDrive.move(x);
			rightDrive.move(y);
		}

		if (controller.get_digital(E_CONTROLLER_DIGITAL_L1))
		{
			if (!drivePressed)
			{
				arcade = !arcade;
				drivePressed = true;
			}
		}
		else
		{
			drivePressed = false;
		}

		// leftDrive.move(controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
		// rightDrive.move(controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
		// chassis.arcade(y, x, 2.7);
		// chassis.curvature(y, x, 2.7);
		// Set intake motor velocity based on button state
		if (controller.get_digital(E_CONTROLLER_DIGITAL_R1))
		{
			intake.move(-127);
			toggleIntake = false;
		}
		else if (controller.get_digital(E_CONTROLLER_DIGITAL_R2))
		{
			intake.move(127);
			toggleIntake = false;
		}
		else if (toggleIntake)
		{
			intake.move(-127);
		}
		else
		{
			intake.brake();
		}

		// Toggle flywheel based on button state
		if (flywheelOn)
		{
			flywheel.move(120);
		}
		else
		{
			flywheel.brake();
		}

		if (controller.get_digital(E_CONTROLLER_DIGITAL_Y))
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
		if (controller.get_digital(E_CONTROLLER_DIGITAL_L2) || controller.get_digital(E_CONTROLLER_DIGITAL_B))
		{
			toggleLift = false;
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

		// Toggle lift based on button state
		if (controller.get_digital(E_CONTROLLER_DIGITAL_X))
		{
			toggleHang = false;
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

		if (toggleHang)
		{
			hang.set_value(1);
		}
		else
		{
			hang.set_value(0);
		}

		if (toggleLift)
		{
			lift.set_value(1);
		}
		else
		{
			lift.set_value(0);
		}

		// toggle wings
		if (controller.get_digital(E_CONTROLLER_DIGITAL_A))
		{
			if (!wingsPressed)
			{
				toggleWings = !toggleWings;
				wingsPressed = true;
			}
		}
		else
		{
			wingsPressed = false;
		}

		if (toggleWings)
		{
			rightwing.set_value(1);
			// rightwing.set_value(1);
		}
		else
		{
			// rightwing.set_value(0);
			rightwing.set_value(0);
		}

		if (controller.get_digital(E_CONTROLLER_DIGITAL_Y))
		{
			toggleIntake = true;
		}
		delay(20); // Typical delay in an opcontrol loop
	}
}