#include "main.h"

// left motor group
pros::MotorGroup left_motor_group({-1, -2, -3}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({4, 5, 6}, pros::MotorGears::blue);

// drivetrain settings
lemlib::Drivetrain
    drivetrain(&left_motor_group,          // left motor group
               &right_motor_group,         // right motor group
               12.5,                       // 12.5 inch track width
               lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
               450,                        // drivetrain rpm is 450
               2                           // horizontal drift is 2 (for now)
    );

// imu
pros::Imu imu(9);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(-20);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_2, +0.3);

// odometry settings
lemlib::OdomSensors
    sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
            nullptr,                  // vertical tracking wheel 2  , none exist
            nullptr,                  // horizontal tracking wheel 1, none exist
            nullptr,                  // horizontal tracking wheel 2, none exist
            &imu                      // inertial sensor
    );

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       10,  // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttle_curve(3,    // joystick deadband out of 127
                   10,   // minimum output where drivetrain will move out of 127
                   1.019 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steer_curve(3,    // joystick deadband out of 127
                10,   // minimum output where drivetrain will move out of 127
                1.019 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,            // odometry sensors
                        &throttle_curve,    // throttle curve
                        &steer_curve        // steer curve
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

std::string thinkOfIt =
    "I'm in the thick of it, everybody knows They know me where it snows, I "
    "skied in and they froze I don't know no nothin' 'bout no ice, I'm just "
    "cold Forty something' milli' subs or so, I've been told From the screen "
    "to "
    "the ring, to the pen, to the king Where's my crown? That's my bling "
    "Always drama when I ring See, I believe that if I see it in my heart "
    "Smash through the ceiling 'cause I'm reaching' for the stars Woah-oh-oh "
    "This is how the story goes Woah-oh-oh I guess this is how the story goes ";

int scrollIndex = 0;
bool pistonState = false;
bool hitlerState = false;

void scrollByOne()
{
  if (scrollIndex == 4)
  {
    char temp = thinkOfIt[0];
    thinkOfIt.erase(thinkOfIt.begin());
    thinkOfIt.push_back(temp);
    std::cout << thinkOfIt << std::endl;
    pros::lcd::print(4, "%s", thinkOfIt.c_str());
    scrollIndex = 0;
  }
  scrollIndex++;

  //  // This line is not needed
}

// initialize function. Runs on program startup
void initialize()
{
  //
  pros::adi::DigitalOut piston('A');
  pros::adi::DigitalOut hitler('B');
  pros::Motor intakeMotor(7, pros::MotorGears::blue);
  pros::Motor armMotor(8, pros::MotorGears::blue);
  armMotor.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
  left_motor_group.set_brake_mode(
      pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
  right_motor_group.set_brake_mode(
      pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
  pros::lcd::initialize(); // initialize brain screen
  pros::lcd::print(0, "calibrateing");
  chassis.calibrate(); // calibrate sensors
  // print position to brain screen
  pros::Task screen_task([&]()
                         {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // xc
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      pros::lcd::print(5, "LOWWWWWW TAPER FADEEEEEEE");
      // pros::lcd::print(3, "Motor pos: %f" ,armMotor.get_position());
      controller.print(0, 0, "kobosh");
      // controller.rumble(". - . -");
      scrollByOne();
      // delay to save resources
      pros::delay(20);
    } });
  // pros::Task brakeBypassHopefully([&]() {
  //   pros::Motor armMotor(8, pros::MotorGears::blue);
  //   while (true) {
  //     armMotor.brake();
  //     // delay to save resources
  //     pros::delay(50);
  //   }
  // });
}

void opcontrol()
{
  pros::adi::DigitalOut piston('A');
  pros::adi::DigitalOut hitler('B');
  pros::Motor intakeMotor(7, pros::MotorGears::blue);
  pros::Motor armMotor(8, pros::MotorGears::blue);
  // loop forever
  while (true)
  {
    // get left y and right x positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    int intakeForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    int intakeBackward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    int armForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    int armBackward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    int pistonToggle = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    int hitlerToggle = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    int leftBtn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    int upBtn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    if (intakeForward)
    {
      intakeMotor.move_velocity(-600);
    }
    else if (intakeBackward)
    {
      intakeMotor.move_velocity(600);
    }
    else
    {
      intakeMotor.move_velocity(0);
    }

    if (armForward)
    {
      armMotor.move_absolute(-2647, 600);
    }
    else if (armBackward)
    {
      armMotor.move_absolute(-100, 600);
    }
    else
    {
      armMotor.move_velocity(0);
    }

    if (pistonToggle)
    {
      if (!pistonState)
      {
        pistonState = true;
        piston.set_value(true);
        while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
        {
          // delay to save resources
          pros::delay(25);
        }
      }
      else
      {
        pistonState = false;
        piston.set_value(false);
        while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
        {
          // delay to save resources
          pros::delay(25);
        }
      }
    }
    if (hitlerToggle)
    {
      if (!hitlerState)
      {
        hitlerState = true;
        hitler.set_value(true);
        while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
          // delay to save resources
          pros::delay(25);
        }
      }
      else
      {
        hitlerState = false;
        hitler.set_value(false);
        while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
          // delay to save resources
          pros::delay(25);
        }
      }
    }
    if (leftBtn)
    {
      autonomous();
      while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
      {
      }
    }

    // if (upBtn) {
    //   lemlib::Pose poseA(-52.774, // x position
    //                      -11.717, // y position
    //                      0);      // heading
    //   chassis.setPose(poseA);
    //   while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
    //   }
    // }

    // move the robot
    chassis.arcade(leftY, rightX);

    // delay to save resources
    pros::delay(25);
  }
}
ASSET(path_jerryio_1_txt);
ASSET(path_jerryio_2_txt);
ASSET(path_jerryio_3_txt);
ASSET(path_jerryio_4_txt);
void autonomous()
{
  pros::adi::DigitalOut piston('A');
  pros::adi::DigitalOut hitler('B');
  pros::Motor intakeMotor(7, pros::MotorGears::blue);
  pros::Motor armMotor(8, pros::MotorGears::blue);
  lemlib::Pose startPose(-52.774, // x position
                         -11.717, // y position
                         90);     // heading
  chassis.setPose(startPose);
  chassis.follow(path_jerryio_1_txt, 10, 1000, false);
  pros::delay(1000);
  piston.set_value(true);
  intakeMotor.move_velocity(-600);
  pros::delay(1000);
  chassis.follow(path_jerryio_2_txt, 10, 1000);
  pros::delay(2000);
  intakeMotor.move_velocity(0);
}