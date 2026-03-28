#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <algorithm>


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-19, 18, -20},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({13, -12, 11},
                            pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor
pros::Imu imu(17);

//intake motors
pros::Motor intakebottom(1, pros::MotorGearset::blue);
pros::Motor intaketop(10, pros::MotorGearset::blue);

//odom
pros::Rotation verticalEncoder(2);

//distance sensors
pros::Distance leftSensor(3);
pros::Distance rightSensor(15);
pros::Distance midSensor(14);

//pnuematics
pros::adi::DigitalOut scraper1('G', false);
pros::adi::DigitalOut scraper2('H', false);
pros::adi::DigitalOut ballblock('E', true);
pros::adi::DigitalOut descore('F', false);

//tracking wheel
lemlib::TrackingWheel verticalWheel(
    &verticalEncoder,
    lemlib::Omniwheel::NEW_2, // replace with wheel size
    0.375f,  // horizontal offset from center (replace)
    -1.75f   // vertical offset from center (replace)
);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              15, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&verticalWheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);



// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, nullptr, nullptr); //prev had &throttleCurve, &steerCurve



//DISTANCE SENSOR CODE

void getSensorLeft(){
    float leftDist = leftSensor.get_distance();
    float newXleft = 56-leftDist-5.75;
    float currentY = chassis.getPose().y;
    float currentTheta = imu.get_heading();

    chassis.setPose(newXleft, currentY, currentTheta);
}

void getSensorRight(){
    float rightDist = rightSensor.get_distance();
    float newXright = 56-rightDist-5.75;
    float currentY = chassis.getPose().y;
    float currentTheta = imu.get_heading();

    chassis.setPose(newXright, currentY, currentTheta);
}

int autonSelection = 0;
const int NUM_AUTONS = 7;

void selectorTask(void*) {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Right 4 Push");

    pros::lcd::register_btn0_cb([]() {
        autonSelection = (autonSelection - 1) % NUM_AUTONS;
        if (autonSelection == 0) pros::lcd::set_text(0, "Right 4 Push");
        else if (autonSelection == 1) pros::lcd::set_text(0, "Right 7 Push");
        else if (autonSelection == 2) pros::lcd::set_text(0, "Right Split 3-4");
        else if (autonSelection == 3) pros::lcd::set_text(0, "Left 4 Push");
        else if (autonSelection == 4) pros::lcd::set_text(0, "Left 7 Push");
        else if (autonSelection == 5) pros::lcd::set_text(0, "Left Split 3-4");
        else if (autonSelection == 6) pros::lcd::set_text(0, "Win Point");
    });
    pros::lcd::register_btn2_cb([]() {
        autonSelection = (autonSelection + 1 + NUM_AUTONS) % NUM_AUTONS;
        if (autonSelection == 0) pros::lcd::set_text(0, "Right 4 Push");
        else if (autonSelection == 1) pros::lcd::set_text(0, "Right 9 Push");
        else if (autonSelection == 2) pros::lcd::set_text(0, "Right Split 4-5");
        else if (autonSelection == 3) pros::lcd::set_text(0, "Left 4 Push");
        else if (autonSelection == 4) pros::lcd::set_text(0, "Left 9 Push");
        else if (autonSelection == 5) pros::lcd::set_text(0, "Left Split 4-5");
        else if (autonSelection == 6) pros::lcd::set_text(0, "Win Point");
    });

    while (true) pros::delay(100);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.ss
 */
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    controller.clear();
    pros::delay(50);
    controller.set_text(0, 0, "有志者事竟成");
    pros::Task selector(selectorTask);
    pros::delay(5000);
    pros::lcd::set_text(0, "Jack is awesome");
    pros::delay(500);
    pros::Task screen_task([&]() {
    while (true) {
    // print robot location to the brain screen
    pros::lcd::print(1, "X: %f", chassis.getPose().x); // x
    pros::lcd::print(2, "Y: %f", chassis.getPose().y); // y
    pros::lcd::print(3, "Theta: %f", chassis.getPose().theta); // heading

    // print measurements from the rotation sensor
    pros::lcd::print(4, "Rotation Sensor: %i", verticalEncoder.get_position());

    //print inertial
    //pros::lcd::print(4, "IMU get heading: %f degrees\n", imu.get_heading());
    pros::lcd::print(5, "IMU: %f", imu.get_heading());

    // delay to save resources
    pros::delay(50);
    }
    });
}

 /** 
*/

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

//ORDER
//.forwards
//.lead
//.maxSpeed
//.minSpeed
//.earlyExitRange

void rightFour(){
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(31, 0, 90, 1500, {.lead = 0.1, .minSpeed = 20, .earlyExitRange = 8});
    chassis.moveToPose(31, -15, 90, 1000, {.lead = 0.1, .maxSpeed = 70, .minSpeed = 40});
    pros::delay(100);
    scraper1.set_value(true);
    scraper2.set_value(true);
    intakebottom.move_velocity(-600);
    chassis.waitUntilDone();
    pros::delay(750);
    chassis.moveToPose(31, 16, 90, 750, {.forwards = false, .minSpeed = 100}, false);
    intaketop.move_velocity(600);
    pros::delay(1000);
    intakebottom.move_velocity(0);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPose(31, 6, 90, 750, {.minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(42, 25, 90, 2000, {.forwards = false, .lead = 0.7, .maxSpeed = 100, .minSpeed = 50});   
}

void rightSeven(){
    //unique code
    chassis.setPose(0, 0, -60);
    intakebottom.move_velocity(-600);
    chassis.moveToPose(10, 30, -60, 1500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.turnToHeading(35, 300);
    chassis.moveToPose(31, 0, 90, 2000, {.lead = 0.1, .minSpeed = 50, .earlyExitRange = 5});
    chassis.moveToPose(31, -15, 90, 1000, {.maxSpeed = 75});
    //right 4 code
    pros::delay(100);
    scraper1.set_value(true);
    scraper2.set_value(true);
    chassis.waitUntilDone();
    pros::delay(750);
    chassis.moveToPose(31, 16, 90, 750, {.forwards = false, .minSpeed = 100}, false);
    intaketop.move_velocity(600);
    pros::delay(1000);
    intakebottom.move_velocity(0);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPose(31, 6, 90, 750, {.minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(42, 25, 90, 2000, {.forwards = false, .lead = 0.7, .maxSpeed = 100, .minSpeed = 50});
}

void rightSplit(){
    //right 7 code
    chassis.setPose(0, 0, -60);
    intakebottom.move_velocity(-600);
    chassis.moveToPose(10, 30, -60, 1500, {.minSpeed = 30, .earlyExitRange = 3});
    //unique
    chassis.turnToHeading(-135, 300);
    chassis.moveToPose(-1, 10, -135, 750, {.maxSpeed = 100});
    chassis.moveToPose(31, 0, -135, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 3});
    chassis.turnToHeading(-270, 300);
    //right 4 code (with -270 instead of 90)
    chassis.moveToPose(31, -15, -270, 1000, {.maxSpeed = 75});
    pros::delay(100);
    scraper1.set_value(true);
    scraper2.set_value(true);
    chassis.waitUntilDone();
    pros::delay(750);
    chassis.moveToPose(31, 16, -270, 750, {.forwards = false, .minSpeed = 100}, false);
    intaketop.move_velocity(600);
    pros::delay(1000);
    intakebottom.move_velocity(0);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPose(31, 6, -270, 750, {.minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(42, 25, -270, 2000, {.forwards = false, .lead = 0.7, .maxSpeed = 100, .minSpeed = 50});
}

void leftFour(){
    //opposite of right 4 (angles)
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(31, 0, -90, 1500, {.lead = 0.1, .minSpeed = 20, .earlyExitRange = 8});
    chassis.moveToPose(31, -15, -90, 1000, {.lead = 0.1, .maxSpeed = 70, .minSpeed = 40});
    pros::delay(100);
    scraper1.set_value(true);
    scraper2.set_value(true);
    intakebottom.move_velocity(-600);
    chassis.waitUntilDone();
    pros::delay(750);
    chassis.moveToPose(31, 16, -90, 750, {.forwards = false, .minSpeed = 100}, false);
    intaketop.move_velocity(600);
    pros::delay(1000);
    intakebottom.move_velocity(0);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPose(31, 6, -90, 750, {.minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(42, 25, -90, 2000, {.forwards = false, .lead = 0.7, .maxSpeed = 100, .minSpeed = 50});
}

void leftSeven(){
    //right 7 reversed
    chassis.setPose(0, 0, 60);
    intakebottom.move_velocity(-600);
    chassis.moveToPose(10, 30, 60, 1500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.turnToHeading(35, 300);
    chassis.moveToPose(31, 0, -90, 2000, {.lead = 0.1, .minSpeed = 50, .earlyExitRange = 5});
    chassis.moveToPose(31, -15, -90, 1000, {.maxSpeed = 75});
    //left 4 code
    pros::delay(100);
    scraper1.set_value(true);
    scraper2.set_value(true);
    intakebottom.move_velocity(-600);
    chassis.waitUntilDone();
    pros::delay(750);
    chassis.moveToPose(31, 16, -90, 750, {.forwards = false, .minSpeed = 100}, false);
    intaketop.move_velocity(600);
    pros::delay(1000);
    intakebottom.move_velocity(0);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPose(31, 6, -90, 750, {.minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(42, 25, -90, 2000, {.forwards = false, .lead = 0.7, .maxSpeed = 100, .minSpeed = 50});
}

void leftSplit(){
    //left 7
    chassis.setPose(0, 0, 60);
    intakebottom.move_velocity(-600);
    chassis.moveToPose(10, 30, 60, 1500, {.minSpeed = 30, .earlyExitRange = 3});
    //unique right split reversed
    chassis.turnToHeading(135, 300);
    chassis.moveToPose(-1, 10, 135, 750, {.maxSpeed = 100});
    chassis.moveToPose(31, 0, 135, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 3});
    chassis.turnToHeading(270, 300);
    //left 4 code (with 270 instead of -90)
    chassis.moveToPose(31, -15, 270, 1000, {.lead = 0.1, .maxSpeed = 70, .minSpeed = 40});
    pros::delay(100);
    scraper1.set_value(true);
    scraper2.set_value(true);
    intakebottom.move_velocity(-600);
    chassis.waitUntilDone();
    pros::delay(750);
    chassis.moveToPose(31, 16, 270, 750, {.forwards = false, .minSpeed = 100}, false);
    intaketop.move_velocity(600);
    pros::delay(1000);
    intakebottom.move_velocity(0);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPose(31, 6, 270, 750, {.minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(42, 25, 270, 2000, {.forwards = false, .lead = 0.7, .maxSpeed = 100, .minSpeed = 50});
}

void winPoint(){

}

void autonomous() {
    if (autonSelection == 0) rightFour();
    else if (autonSelection == 1) rightSeven();
    else if (autonSelection == 2) rightSplit();
    else if (autonSelection == 3) leftFour();
    else if (autonSelection == 4) leftSeven();
    else if (autonSelection == 5) leftSplit();
    else if (autonSelection == 6) winPoint();
    
}








/**
 * Runs in driver control
 */
float cubicDrive(float input, float scaling = 1.0f) {
    const float maxInput = 127.0f;
    return scaling * (input * input * input) / (maxInput * maxInput);
}

bool scraper1out = false;
bool scraper2out = false;
bool descoreUp = false;

void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        float throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        float turn     = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        float leftPower  = cubicDrive(throttle) + cubicDrive(turn);
        float rightPower = cubicDrive(throttle) - cubicDrive(turn);

        if (leftPower  >  127.0f) leftPower  =  127.0f;
        if (leftPower  < -127.0f) leftPower  = -127.0f;
        if (rightPower >  127.0f) rightPower =  127.0f;
        if (rightPower < -127.0f) rightPower = -127.0f;

        leftMotors.move(leftPower);
        rightMotors.move(rightPower);
        
        //top goal
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intakebottom.move_velocity(-600);
            intaketop.move_velocity(600);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intakebottom.move_velocity(600);
            intaketop.move_velocity(-600);
                }

        //middle goal
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            intakebottom.move_velocity(-600);
            intaketop.move_velocity(0);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            intakebottom.move_velocity(-87);
            intaketop.move_velocity(87);
            ballblock.set_value(false);
        }
        else{
            intakebottom.move_velocity(0);
            intaketop.move_velocity(0);
            ballblock.set_value(true);
        }


        
        //scraper
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            scraper1out = !scraper1out;
            scraper2out = !scraper2out;
            scraper1.set_value(scraper1out);
            scraper2.set_value(scraper2out);
        }

        //flap
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            descoreUp = !descoreUp;
            descore.set_value(descoreUp);
        }
  

        //delay
        pros::delay(10);
    }
}