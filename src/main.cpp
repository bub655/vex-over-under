#include "main.h"
#include "lemlib/api.hpp"
#include <string>
using namespace pros;

Controller controller(E_CONTROLLER_MASTER);
Motor motor_lf(4, E_MOTOR_GEARSET_06, true);
Motor motor_lb(5, E_MOTOR_GEARSET_06, true);
Motor motor_lt(6, E_MOTOR_GEARSET_06, false);
Motor motor_rf(7, E_MOTOR_GEARSET_06, false);
Motor motor_rb(8, E_MOTOR_GEARSET_06, false);
Motor motor_rt(10, E_MOTOR_GEARSET_06, true);
Motor intake(14);
Motor flywheel(11, true);
Motor_Group leftDrive({motor_lf, motor_lb, motor_lt});
Motor_Group rightDrive({motor_rf, motor_rb, motor_rt});
ADIDigitalOut lift('A');
ADIDigitalOut hang('B');
ADIDigitalOut wings('C');
Imu iner(13);

lemlib::Drivetrain_t drivetrain{
		&leftDrive,	 // left drivetrain motors
		&rightDrive, // right drivetrain motors
		11.4,				 // track width
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
		0.1, // kP
		0.1, // kD
		1,	 // smallErrorRange
		100, // smallErrorTimeout
		3,	 // largeErrorRange
		500, // largeErrorTimeout
		5		 // slew rate
};

lemlib::ChassisController_t angularController{
		2,	 // kP
		1,	 // kD
		1,	 // smallErrorRange
		100, // smallErrorTimeout
		3,	 // largeErrorRange
		500, // largeErrorTimeout
		5		 // slew rate
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
		lemlib::Pose pose = chassis.getPose();		// get the current position of the robot
		lcd::print(0, "x: %f", pose.x);						// print the x position
		lcd::print(1, "y: %f", pose.y);						// print the y position
		lcd::print(2, "heading: %f", pose.theta); // print the heading
		delay(10);
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
	Task screenTask();
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
	// flywheel.move(115);
	// leftDrive.move(-127);
	// rightDrive.move(127);
	// delay(100);
	// leftDrive.move(0);
	// rightDrive.move(0);
	// delay(36000);
	chassis.setPose(-48.95579437229437, -59.557686147186146, 45);
	chassis.follow("skills_p1.txt", 4500, 12);
	leftDrive.move(127);
	rightDrive.move(127);
	delay(600);
	leftDrive.move(-127);
	rightDrive.move(-127);
	delay(100);
	chassis.follow("skills_p2.txt", 4000, 10, true);
	chassis.follow("skills_p3.txt", 1000, 10);
	chassis.follow("skills_p4.txt", 4000, 10, true);
	chassis.follow("skills_p5.txt", 1500, 10);
	chassis.follow("skills_p6.txt", 5000, 10, true);
	leftDrive.move(-127);
	rightDrive.move(-127);
	delay(800);
	leftDrive.move(0);
	rightDrive.move(0);
}

void nearSide()
{
	chassis.setPose(-43.135147907647905, -54.152800144300144, 20);
	intake.move(127);
	chassis.follow("near_p1.txt", 2500, 12);
	// leftDrive.move(-127);
	// delay(500);
	chassis.follow("near_p2.txt", 2800, 15, true);
	intake.move(0);
	wings.set_value(1);
	delay(250);
	leftDrive.move(-127);
	rightDrive.move(127);
	delay(330);
	wings.set_value(0);
	chassis.follow("near_p3.txt", 3000, 10, true);
	leftDrive.move(50);
	rightDrive.move(50);
	delay(100);
	leftDrive.move(0);
	rightDrive.move(0);
	// leftDrive.move(127);
	// rightDrive.move(127);
	// delay(200);
	// leftDrive.move(-127);
	// rightDrive.move(-127);
	// delay(100);
	// chassis.follow("near_4.txt", 2000, 10, true);
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

void autonomous()
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
	float power, turn;
	bool buttonR1Pressed, buttonR2Pressed, buttonYPressed, buttonXPressed, buttonL2Pressed, buttonAPressed;
	bool flywheelOn, flywheelPressed = false;
	bool toggleLift, liftPressed = false;
	bool toggleHang, hangPressed = false;
	bool toggleWings, wingsPressed = false;
	while (true)
	{
		// Task screenTask(screen);
		// Get joystick values
		turn = 0.5 * (controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X));
		power = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

		// Check if buttons are pressed
		buttonR1Pressed = controller.get_digital(E_CONTROLLER_DIGITAL_R1);
		buttonR2Pressed = controller.get_digital(E_CONTROLLER_DIGITAL_R2);
		buttonYPressed = controller.get_digital(E_CONTROLLER_DIGITAL_Y);
		buttonXPressed = controller.get_digital(E_CONTROLLER_DIGITAL_X);
		buttonL2Pressed = controller.get_digital(E_CONTROLLER_DIGITAL_L2);
		buttonAPressed = controller.get_digital(E_CONTROLLER_DIGITAL_A);
		// Set motor velocities based on joystick inputs and button state
		leftDrive.move((turn + power));
		rightDrive.move((power - turn));
		// Set intake motor velocity based on button state
		if (buttonR1Pressed == 1)
		{
			intake.move(127);
		}
		else if (buttonR2Pressed == 1)
		{
			intake.move(-127);
		}
		else
		{
			intake.brake();
		}

		if (buttonL2Pressed)
		{
			flywheel.move(-127);
		}

		// Toggle flywheel based on button state
		if (flywheelOn)
		{
			flywheel.move(115);
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
		if (buttonL2Pressed)
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

		// toggle wings
		if (buttonAPressed)
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
			wings.set_value(1);
		}
		else
		{
			wings.set_value(0);
		}
		delay(20); // Typical delay in an opcontrol loop
	}
}