#include "main.h"
#include "lemlib/api.hpp"
#include <string>
#include <bits/stdc++.h>
using namespace pros;
ASSET(bowl_txt);
ASSET(first_txt);
ASSET(second_txt);
ASSET(third_txt);

Controller controller(E_CONTROLLER_MASTER);
Motor motor_lf(4, E_MOTOR_GEARSET_06, true);
Motor motor_lb(5, E_MOTOR_GEARSET_06, true);
Motor motor_lt(6, E_MOTOR_GEARSET_06, false);
Motor motor_rf(7, E_MOTOR_GEARSET_06, false);
Motor motor_rb(8, E_MOTOR_GEARSET_06, false);
Motor motor_rt(10, E_MOTOR_GEARSET_06, true);
Motor intake(14, true);
Motor flywheel(11, true);
Motor_Group leftDrive({motor_lf, motor_lb, motor_lt});
Motor_Group rightDrive({motor_rf, motor_rb, motor_rt});
ADIDigitalOut lift('A');
ADIDigitalOut hang('B');
ADIDigitalOut leftwing('C');
ADIDigitalOut rightwing('H');
Imu iner(13);

lemlib::Drivetrain drivetrain{
		&leftDrive,	 // left drivetrain motors
		&rightDrive, // right drivetrain motors
		11.4,				 // track width
		3.25,				 // wheel rpm
		400,				 // gain
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
		14.5, // kP
		0,		// kI
		30,		// kD
		10,		// windupRange
		0.25, // smallErrorRange
		300,	// smallErrorTimeout
		3,		// largeErrorRange
		500,	// largeErrorTimeout
		100		// slew rate
};

// turning PID
lemlib::ControllerSettings angularSettings{
		11,	 // kP
		0,	 // kI
		94,	 // kD
		10,	 // windupRange
		1,	 // smallErrorRange
		300, // smallErrorTimeout
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
	motor_lb.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_lf.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_lt.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_rb.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_rf.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_rt.set_brake_mode(E_MOTOR_BRAKE_COAST);
	flywheel.set_brake_mode(E_MOTOR_BRAKE_COAST);
	intake.set_brake_mode(E_MOTOR_BRAKE_HOLD);
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
	flywheel.set_brake_mode(MOTOR_BRAKE_COAST);
	flywheel.move(120);
	chassis.setPose(0, 0, -30);
	chassis.moveToPoint(-20, 32, 1200);
	chassis.waitUntilDone();
	chassis.moveToPose(-20, 10, 70, 1750, {forwards : false});
	delay(500);
	lift.set_value(1);
	chassis.waitUntilDone();
	delay(35000);
	flywheel.brake();
	lift.set_value(0);
	chassis.setPose(-58.071, -45.246, 70);
	chassis.follow(bowl_txt, 12, 4250);
	chassis.waitUntilDone();
	leftDrive.move(127);
	rightDrive.move(127);
	delay(500);

	chassis.follow(first_txt, 12, 4000, false);
	delay(1000);
	leftwing.set_value(1);
	rightwing.set_value(1);
	chassis.waitUntilDone();
	leftDrive.move(-127);
	rightDrive.move(-127);
	delay(500);
	leftwing.set_value(0);
	rightwing.set_value(0);
	chassis.follow(second_txt, 12, 2000);
	chassis.waitUntilDone();
	leftwing.set_value(1);
	rightwing.set_value(1);
	chassis.turnTo(1.326, 48.1, 750);
	chassis.waitUntilDone();
	chassis.follow(third_txt, 12, 2000, false);
	chassis.waitUntilDone();
	leftDrive.move(-127);
	rightDrive.move(-127);
	delay(500);
}

void skills_old()
{
	// flywheel.move(115);
	// leftDrive.move(-127);
	// rightDrive.move(127);
	// delay(100);
	// leftDrive.move(0);
	// rightDrive.move(0);
	// delay(36000);

	// chassis.setPose(0, 0, 0);
	// chassis.turnTo(5, 5, 5000);

	// chassis.setPose(-48.95579437229437, -59.557686147186146, 45);
	// chassis.follow("skills_p1.txt", 4000, 10, true);
	chassis.setPose(-49.7873152958153, -56.64736291486291, 45);
	chassis.follow(bowl_txt, 4500, 12);
	// leftDrive.move(127);
	// rightDrive.move(127);
	// delay(600);
	// leftDrive.move(-127);
	// rightDrive.move(-127);
	// delay(100);
	// chassis.follow("skills_p2.txt", 4000, 10, true);
	// chassis.follow("skills_p3.txt", 1000, 10);
	// chassis.follow("skills_p4.txt", 4000, 10, true);
	// chassis.follow("skills_p5.txt", 1500, 10);
	// chassis.follow("skills_p6.txt", 5000, 10, true);
	// leftDrive.move(-127);
	// rightDrive.move(-127);
	// delay(800);
	// leftDrive.move(0);
	// rightDrive.move(0);
}

void farSide()
{
	leftDrive.move(-127);
	rightDrive.move(-127);
	delay(3000);
	// leftDrive.move(127);
	// rightDrive.move(127);
	// delay(1000);
	// leftDrive.move(-127);
	// rightDrive.move(-127);
	// delay(3000);
	leftDrive.move(0);
	rightDrive.move(0);
}
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

void wp_5ball()
{
	// BALL 1
	intake.move(127);
	chassis.setPose(0, 0, 0);
	chassis.moveToPose(-9, 51, -15, 1400);
	chassis.waitUntilDone();
	chassis.turnTo(20, 51, 650);
	chassis.waitUntilDone();
	intake.move(-127);
	chassis.moveToPoint(16, 51, 550);
	chassis.moveToPoint(0, 51, 500, false);
	// // BALL 2
	// intake.move(127);
	// chassis.moveToPose(-35, 60, -90, 1000);
	// chassis.waitUntilDone();
	// delay(300);
	// chassis.moveToPose(0, 56, 90, 750);
	// intake.move(-127);
	// chassis.moveToPoint(14, 56, 750);
	// chassis.moveToPoint(0, 55, 500, false);

	//  BALL 3
	chassis.waitUntilDone();

	intake.move(127);
	chassis.moveToPose(-35.75, 31.5, -90, 1850);
	chassis.waitUntilDone();
	delay(200);
	intake.move(0);
	chassis.moveToPose(-2, -5, 50, 1750, {forwards : false});
	chassis.waitUntilDone();
	intake.move(-127);
	delay(300);
	chassis.turnTo(-36, -3.5, 650);
	chassis.waitUntilDone();
	intake.move(127);
	chassis.moveToPoint(-28, -3.5, 1000);
	chassis.waitUntilDone();

	chassis.moveToPoint(-4, -2, 1000, false);
	chassis.waitUntilDone();
	leftwing.set_value(1);
	delay(50);
	chassis.moveToPose(17, 12, 180, 2000, {forwards : false});
	chassis.moveToPoint(20, 26, 750, false);
	chassis.moveToPoint(20, 12, 500);
}

void disrupt_wp()
{
	// steal ball
	intake.move(127);
	chassis.setPose(0, 0, 10);
	chassis.moveToPoint(8.7, 47, 1250);
	chassis.waitUntilDone();
	chassis.moveToPoint(-2, 0, 1500, false);
	// alliance ball score
	chassis.turnTo(10, 0, 800);
	chassis.waitUntilDone();
	intake.move(-50);
	delay(200);
	chassis.turnTo(-10, 0, 800);
	chassis.waitUntilDone();
	intake.move(0);
	chassis.moveToPose(-20, 21, -10, 1700);
	chassis.waitUntilDone();
	leftDrive.move(127);
	rightDrive.move(127);
	delay(300);
	// dematchload
	chassis.moveToPose(-4, -2, -90, 3000, {forwards : false});
	delay(300);
	leftwing.set_value(1);
	chassis.waitUntilDone();
	leftDrive.move(-127);
	rightDrive.move(-127);
	delay(200);
	// touch bar
	leftDrive.move(-75);
	rightDrive.move(75);
	delay(200);
	chassis.turnTo(10, -7, 800);
	leftwing.set_value(0);
	chassis.moveToPose(32, -10, 90, 4200);
	chassis.waitUntilDone();
	intake.move(-127);
	rightDrive.move(0);
	leftDrive.move(0);
	delay(500);
	intake.move(0);
}
void autonomous()
{
	disrupt_wp();
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

	motor_lb.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_lf.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_lt.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_rb.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_rf.set_brake_mode(E_MOTOR_BRAKE_COAST);
	motor_rt.set_brake_mode(E_MOTOR_BRAKE_COAST);
	float power, turn;
	bool flywheelOn, flywheelPressed = false;
	bool toggleLift, liftPressed = false;
	bool toggleHang, hangPressed = false;
	bool toggleWings, wingsPressed = false;
	float x, y;
	while (true)
	{

		// Task screenTask(screen)
		// Get joystick values
		x = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
		y = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		turn = 0.6 * (controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X) * pow(fabs(controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X)) / (127), 2));
		power = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) * pow(fabs(controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) / (127), 2);
		// chassis.arcade(y, x, 2.7);
		// Set motor velocities based on joystick inputs and button state
		leftDrive.move((turn + power));
		rightDrive.move((power - turn));
		// chassis.arcade(y, x, 2.7);
		// chassis.curvature(y, x, 2.7);
		// Set intake motor velocity based on button state
		if (controller.get_digital(E_CONTROLLER_DIGITAL_R1))
		{
			intake.move(127);
		}
		else if (controller.get_digital(E_CONTROLLER_DIGITAL_R2))
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
			leftwing.set_value(1);
			rightwing.set_value(1);
		}
		else
		{
			rightwing.set_value(0);
			leftwing.set_value(0);
		}
		delay(20); // Typical delay in an opcontrol loop
	}
}