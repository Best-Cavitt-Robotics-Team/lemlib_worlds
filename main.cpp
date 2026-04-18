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

//pnuematics
pros::adi::DigitalOut scraper1('G', false);
pros::adi::DigitalOut scraper2('H', false);
pros::adi::DigitalOut ballblock('F', true);
pros::adi::DigitalOut descore('E', false);
pros::adi::DigitalOut midDescore('D', false);

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
lemlib::ControllerSettings linearController(7, //10 proportional gain (kP)
                                            0, // integral gain (kI)
                                            10, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.7, //1.7 // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, //10 // derivative gain (kD)
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

//selector

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
        else if (autonSelection == 1) pros::lcd::set_text(0, "Right 7 Push");
        else if (autonSelection == 2) pros::lcd::set_text(0, "Right Split 3-4");
        else if (autonSelection == 3) pros::lcd::set_text(0, "Left 4 Push");
        else if (autonSelection == 4) pros::lcd::set_text(0, "Left 7 Push");
        else if (autonSelection == 5) pros::lcd::set_text(0, "Left Split 3-4");
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
    pros::delay(50);
    controller.set_text(0, 0, "有志者事竟成"); 
    pros::Task screen_task([&]() {
    // while (true) {
    // // print robot location to the brain screen
    // pros::lcd::print(1, "X: %f", chassis.getPose().x); // x
    // pros::lcd::print(2, "Y: %f", chassis.getPose().y); // y
    // pros::lcd::print(3, "Theta: %f", chassis.getPose().theta); // heading

    // // print measurements from the rotation sensor
    // pros::lcd::print(4, "Rotation Sensor: %i", verticalEncoder.get_position());

    // //print inertial
    // //pros::lcd::print(4, "IMU get heading: %f degrees\n", imu.get_heading());
    // pros::lcd::print(5, "IMU: %f", imu.get_heading());

    // // delay to save resources
    // pros::delay(50);
    // }
    });
    // pros::delay(50);
    pros::Task selector(selectorTask);
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
//.horizontalDrift
//.lead
//.maxSpeed
//.minSpeed
//.earlyExitRange

void rightFour(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0,0,90);
    chassis.moveToPoint(17.5, 0, 1500, {.minSpeed = 50, .earlyExitRange = 5}); //180
    chassis.turnToHeading(180, 250, {.minSpeed = 40});
    chassis.moveToPose(17.5, -12, 180, 1000, {.lead = 0, .maxSpeed = 70, .minSpeed = 40}); //18//14
    pros::delay(50); //pev 100
    scraper1.set_value(true);
    scraper2.set_value(true);
    intakebottom.move_velocity(-600);
    descore.set_value(true);
    // chassis.waitUntilDone();
    pros::delay(1100); //tune //pev 1000
    chassis.moveToPose(17.5, 16, 180, 750, {.forwards = false, .lead = 0.5, .maxSpeed = 100, .minSpeed = 70}, false);
    intaketop.move_velocity(600);
    pros::delay(1200); //1000
    descore.set_value(false);
    intakebottom.move_velocity(0);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPose(18, 0, 180, 750, {.lead = 0, .minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(23, 19, 180, 2000, {.forwards = false, .horizontalDrift = 4, .lead = 0.6, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 8}); //prev 23,21.5  
    chassis.turnToHeading(180, 250); 
    
}

void rightSeven(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0, 0, 0);
    intakebottom.move_velocity(-600);
    chassis.moveToPose(9, 17, 30, 1500, {.horizontalDrift = 2, .lead = 0.7, .minSpeed = 50, .earlyExitRange = 3}); //9,17,7,45
    chassis.turnToHeading(135, 500);//130
    chassis.moveToPoint(22, 2, 1700, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.turnToHeading(180, 300, {.minSpeed = 30, .earlyExitRange = 3}); //50
    chassis.moveToPose(21, -17, 180, 1000, {.maxSpeed = 75, .minSpeed = 50});
    pros::delay(100);
    scraper1.set_value(true);
    scraper2.set_value(true);
    descore.set_value(true);
    //left 4 code (skeleton)
    pros::delay(1400);
    chassis.moveToPoint(20.5, 14, 750, {.forwards = false, .maxSpeed = 100, .minSpeed = 80}, false); //l 0
    intaketop.move_velocity(600);
    pros::delay(3000);
    descore.set_value(false);
    intakebottom.move_velocity(0);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPose(20.5, 0, 180, 750, {.lead = 0, .minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(25, 20, 180, 2000, {.forwards = false, .horizontalDrift = 4, .lead = 0.6, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 5}); //19, 0.6
    chassis.turnToHeading(180, 250);
}

void rightSplit(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0, 0, 0);
    intakebottom.move_velocity(-600);
    chassis.moveToPose(9, 17, 30, 1500, {.horizontalDrift = 2, .lead = 0.7, .minSpeed = 50, .earlyExitRange = 3}); //9,17,7,45
    chassis.turnToHeading(-45, 500);
    chassis.moveToPoint(-1, 18, 1700, {.minSpeed = 50}, false);
    midDescore.set_value(true);
    intakebottom.move_velocity(600);
    pros::delay(1500);
    midDescore.set_value(false);
    intakebottom.move_velocity(-600);
    chassis.turnToHeading(-40, 250);
    chassis.moveToPoint(18, 2, 1700, {.forwards = false, .minSpeed = 50, .earlyExitRange = 5});
    chassis.turnToHeading(-180, 300, {.minSpeed = 30, .earlyExitRange = 3}); //50
    pros::delay(200);
    scraper1.set_value(true);
    scraper2.set_value(true);
    chassis.moveToPose(17, -16.5, -180, 1000, {.maxSpeed = 60, .minSpeed = 40}); //75, 40
    descore.set_value(false);
    //left 4 code (skeleton)
    pros::delay(1500);
    chassis.moveToPoint(19, 14, 750, {.forwards = false, .maxSpeed = 100, .minSpeed = 80}, false); //l 0
    intaketop.move_velocity(600);
    pros::delay(1500);
    intakebottom.move_velocity(0);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPose(20, 0, -180, 750, {.lead = 0, .minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(23, 20, -180, 2000, {.forwards = false, .horizontalDrift = 4, .lead = 0.7, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 5}); //19, 0.6
    chassis.turnToHeading(-180, 250, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.moveToPoint(23, 25, 500, {.forwards = false});
}

void leftFour(){
    //opposite of right 4 (angles)
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0,0,-90);
    chassis.moveToPoint(-18, 0, 1500, {.minSpeed = 50, .earlyExitRange = 5}); //180 17
    chassis.turnToHeading(-180, 250, {.minSpeed = 40});
    chassis.moveToPose(-18, -12, -180, 1000, {.lead = 0, .maxSpeed = 70, .minSpeed = 40}); //18
    pros::delay(50); //pev 100
    scraper1.set_value(true);
    scraper2.set_value(true);
    intakebottom.move_velocity(-600);
    descore.set_value(true);
    pros::delay(1100); //tune //pev 750
    chassis.moveToPose(-17, 16, -180, 1000, {.forwards = false, .lead = 0.5, .maxSpeed = 100, .minSpeed = 70}, false);
    intaketop.move_velocity(600);
    pros::delay(1200);
    descore.set_value(false);
    intakebottom.move_velocity(0);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPose(-17, 0, -180, 750, {.lead = 0, .minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(-12, 19, -180, 2000, {.forwards = false, .horizontalDrift = 4, .lead = 0.6, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 8}); //prev 23,21.5  
    chassis.turnToHeading(-180, 250);
    chassis.moveToPoint(-12, 21, 500);
}

void leftSeven(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0, 0, 0);
    intakebottom.move_velocity(-600);
    chassis.moveToPose(-9, 17, -30, 1500, {.horizontalDrift = 2, .lead = 0.7, .minSpeed = 50, .earlyExitRange = 3}); //7,10
    chassis.turnToHeading(-130, 500);
    chassis.moveToPoint(-20, 0, 1700, {.minSpeed = 50, .earlyExitRange = 5}); //eer 5 //ms = 75 //2000 //hd = 2 //21
    chassis.turnToHeading(-180, 300, {.minSpeed = 30, .earlyExitRange = 3}); //50
    chassis.moveToPose(-19, -14, -180, 1000, {.maxSpeed = 75, .minSpeed = 50});
    pros::delay(100);
    scraper1.set_value(true);
    scraper2.set_value(true);
    descore.set_value(true);
    //left 4 code (skeleton)
    pros::delay(1100);
    chassis.moveToPoint(-20.5, 14, 750, {.forwards = false, .maxSpeed = 100, .minSpeed = 80}, false); //l 0
    intaketop.move_velocity(600);
    pros::delay(2000);
    descore.set_value(false);
    intakebottom.move_velocity(0);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPose(-20, 0, -180, 750, {.lead = 0, .minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(-14, 19, -180, 2000, {.forwards = false, .horizontalDrift = 4, .lead = 0.6, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 5}); //14
    chassis.turnToHeading(-180, 250);    
}

void leftSplit(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0, 0, 0);
    intakebottom.move_velocity(-600);
    chassis.moveToPose(-9, 17, -30, 1500, {.horizontalDrift = 2, .lead = 0.6, .minSpeed = 40, .earlyExitRange = 3}); //-9,17, //ms50//7
    chassis.turnToHeading(-130, 500);
    chassis.moveToPose(5, 23, -130, 1000, {.forwards = false, .lead = 0.6, .minSpeed = 50}, false);//.5
    chassis.waitUntilDone();
    ballblock.set_value(false);
    intaketop.move_velocity(300);
    descore.set_value(true);
    pros::delay(1000);
    intaketop.move_velocity(0);
    // chassis.turnToHeading(-140, 250);
    chassis.turnToPoint(-20, 0, 250);
    chassis.waitUntilDone();
    chassis.moveToPoint(-20, 0, 1700, {.maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 5}); //eer 5 //ms = 75 //2000 //hd = 2 //20.5
    chassis.turnToHeading(-180, 300, {.minSpeed = 30, .earlyExitRange = 3}); //50
    chassis.moveToPose(-19, -13, -180, 1000, {.maxSpeed = 75, .minSpeed = 50});//14
    pros::delay(100);
    scraper1.set_value(true);
    scraper2.set_value(true);
    ballblock.set_value(true);
    descore.set_value(false);
    chassis.waitUntilDone();
    //left 4 code (skeleton)
    pros::delay(1500);//1200
    chassis.moveToPoint(-19, 13, 750, {.forwards = false, .maxSpeed = 100, .minSpeed = 80}, false); //l 0 //14
    intaketop.move_velocity(600);
    pros::delay(1500);
    intakebottom.move_velocity(0);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPose(-19, 0, -180, 750, {.lead = 0, .minSpeed = 50, .earlyExitRange = 3});
    chassis.moveToPose(-13, 19, -180, 2000, {.forwards = false, .horizontalDrift = 4, .lead = 0.6, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 5}); //14
    chassis.turnToHeading(-180, 250);
}

void winPoint(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0,0,90);
    chassis.moveToPoint(17.5, 0, 1500, {.minSpeed = 50, .earlyExitRange = 5}); //180
    chassis.turnToHeading(180, 250, {.minSpeed = 40});
    chassis.moveToPose(17.5, -12, 180, 1000, {.lead = 0, .maxSpeed = 70, .minSpeed = 40}); //18//12
    pros::delay(50); //pev 100
    scraper1.set_value(true);
    scraper2.set_value(true);
    intakebottom.move_velocity(-600);
    descore.set_value(false);
    pros::delay(1200); //tune //pev 1100
    chassis.moveToPose(17.5, 17, 180, 750, {.forwards = false, .lead = 0.5, .maxSpeed = 100, .minSpeed = 70}, false);
    intaketop.move_velocity(600);
    pros::delay(1200); //1000
    descore.set_value(true);
    intaketop.move_velocity(0);
    scraper1.set_value(false);
    scraper2.set_value(false);
    chassis.moveToPoint(18, 7, 750, {.minSpeed = 50, .earlyExitRange = 3});
    chassis.turnToHeading(305, 250, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(5, 12, 1000, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.turnToHeading(270, 250, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(-26, 13, 2000, {.minSpeed = 30, .earlyExitRange = 3});//23
    chassis.turnToHeading(242, 250, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(-17, 19, 1500, {.forwards = false}, false);
    chassis.waitUntilDone();
    ballblock.set_value(false);
    intaketop.move_velocity(600);
    pros::delay(1500);

    intaketop.move_velocity(0);
    chassis.moveToPoint(-38, 0, 1700, {.maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 5});
    chassis.turnToHeading(180, 300, {.minSpeed = 30, .earlyExitRange = 3}); //50
    chassis.moveToPose(-37, -16, -180, 1000, {.maxSpeed = 75, .minSpeed = 50});//13.5
    pros::delay(100);
    scraper1.set_value(true);
    scraper2.set_value(true);
    ballblock.set_value(true);
    //left 4 code (skeleton)
    pros::delay(1300);//1200
    chassis.moveToPoint(-38, 16, 750, {.forwards = false, .maxSpeed = 100, .minSpeed = 80}, false); //l 0 //14
    intaketop.move_velocity(600);
    // pros::delay(1500);
    // intakebottom.move_velocity(0);
    // intaketop.move_velocity(0);
    // scraper1.set_value(false);
    // scraper2.set_value(false);
    // chassis.moveToPose(-37, 0, -180, 750, {.lead = 0, .minSpeed = 50, .earlyExitRange = 3});
    // chassis.moveToPose(-33, 19, -180, 2000, {.forwards = false, .horizontalDrift = 4, .lead = 0.6, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 5}); //14
    // chassis.turnToHeading(180, 250);
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
bool midD = false;

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
            intakebottom.move_velocity(-600);//276
            intaketop.move_velocity(600);
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

        //descore
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            descoreUp = !descoreUp;
            descore.set_value(descoreUp);
        }
  
        //middle goal descore
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            midD = !midD;
            midDescore.set_value(midD);
        }
        //delay
        pros::delay(10);
    }
}