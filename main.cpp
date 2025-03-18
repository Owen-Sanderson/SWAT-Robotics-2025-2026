#include "main.h"

/**Below is the #include for lemlib, an open source odometry and PID library that will be
 used to generate autonomous programs in this project, please DO NOT delete this line of code, for any
 references to the lemlib api and setup for most of this project please
 refer to this link: https://lemlib.readthedocs.io/en/stable/tutorials/1_getting_started.html */
#include "lemlib/api.hpp" // IWYU pragma: keep

//Drivetrain Configuration
pros::MotorGroup left_motors({1, 2, 3}, pros::MotorGearset::blue); 
pros::MotorGroup right_motors({4, 5, 6}, pros::MotorGearset::blue); 

pros::v5::motor leftA(1, pros::MotorGearset::blue);

//Drivetrain Class (Contains all Drivtrain Settings)
	lemlib::Drivetrain drivetrain(&left_motors, 
							  &right_motors,
							  10, //track width in inches
							  lemlib::Omniwheel::NEW_325, //wheel size
							  450, //drivetrain rpm
							  2 //horizontal drift
	);

//IMU Configuration
pros::Imu imu(10);

//Rotation Sensor Configurations
pros::Rotation horizontal_sensor(1); // x axis sensor 
pros::Rotation vertical_sensor(2); // y axis sensor

//Tracking Wheel Configuration
//(sensor-name, omniwheel size, offsets, gear-ratio);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_2, 0, 1);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_2, 0, 1);

//Odom Config
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
	nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
	&horizontal_tracking_wheel, // horizontal tracking wheel 1
	nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
	&imu // inertial sensor
);

//PID Controllers

	// lateral PID controller
	lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
	);

	// angular PID controller
	lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
	);

// create the chassis
lemlib::Chassis chassis(drivetrain,
	lateral_controller,
	angular_controller,
	sensors
);


/** Example Function for LLEMU button callbacks:

		void on_center_button() {
			static bool pressed = false;
			pressed = !pressed;
			if (pressed) {
				pros::lcd::set_text(2, "I was pressed!");
			} else {
				pros::lcd::clear_line(2);
			}
}

Buttons for LLEMU are numbered 0-2, right-left, register the desired button inside the "initialize" function
and assign a callback function inside the parentheses following the example shown above.
*/

//autonomous selector configuration
int auton;
bool autonStarted;


// Below is the initialize function, this runs automatically when the robot is started
void initialize() {
	//initialize lcd display on the v5 brain
	pros::lcd::initialize();
	chassis.calibrate(); //calibrate sensors
	pros::Task screen_task([&](){
		while  (true) {
			//display coordinates to the brain screen 
			pros::lcd::print(0, "X: %f", chassis.getPose().x);
			pros::lcd::print(1, "Y: %f", chassis.getPose().y);
			pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
			
			//display autonomous selected
			pros::lcd::print(4, "Autonomous Selected:");

			//display motor temps
			pros::lcd::print(6,leftA.get_temperature());



	//Assign each button of the LLEMU callback functions
	  //Left Button
		pros::lcd::register_btn0_cb();
	  //Middle Button
		pros::lcd::register_btn1_cb();
	  //Right Button
		pros::lcd::register_btn2_cb();

	pros::delay(15);
	}
  });
}



void disabled() {}

// runs after initialize, when connected to field control
void competition_initialize(){}


void autonomous() {}


pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
	while (true){

		//get joystick position
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

		//move the robot
		chassis.arcade(leftX, leftY);

		pros::delay(20);
	}
}