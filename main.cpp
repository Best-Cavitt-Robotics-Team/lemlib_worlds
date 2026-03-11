#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <algorithm>


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-7, 6, -5},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({10, -9, 8}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 19
pros::Imu imu(19);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

//intake motors
pros::Motor intakebottom(1, pros::MotorGearset::blue);
pros::Motor intaketop(2, pros::MotorGearset::blue);

//distance sensors
pros::Distance frontDist(10); // change ports to match yours
pros::Distance backDist(11);
pros::Distance leftDist(12);
pros::Distance rightDist(13);

//pnuematics
pros::adi::DigitalOut scraper1('G', false);
pros::adi::DigitalOut scraper2('H', false);
pros::adi::DigitalOut ballblock('E', false);
pros::adi::DigitalOut descore('F', false);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
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
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
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
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

//DISTANCE SENSOR CODE

// Convert mm to inches
float distInches(pros::Distance& sensor) {
    return sensor.get() / 25.4f;
}

// ── POSITION CORRECTION ──────────────────────────────
// If you know the field is 144x144 inches (VEX full field)
// you can calculate your X and Y from opposite sensor pairs
float getXFromSensors() {
    float left  = distInches(leftDist);
    float right = distInches(rightDist);
    // X position from left wall = left sensor reading
    // cross-check: left + right should equal field width (72" for half field)
    return left; 
}

float getYFromSensors() {
    float front = distInches(frontDist);
    float back  = distInches(backDist);
    return back; // Y position from back wall
}

// ── ANGULAR CORRECTION ───────────────────────────────
// Uses front+back OR left+right pair to find angle offset
// If perfectly straight, both sides of a pair read the same
float getAngleFromLR() {
    float left  = distInches(leftDist);
    float right = distInches(rightDist);
    // robot width (inches) — measure your robot
    const float robotWidth = 14.0f;
    return atan2f(left - right, robotWidth) * (180.0f / M_PI);
}

float getAngleFromFB() {
    float front = distInches(frontDist);
    float back  = distInches(backDist);
    const float robotLength = 14.0f; // measure your robot
    return atan2f(front - back, robotLength) * (180.0f / M_PI);
}

void sensorCorrectionFn() {
    while (true) {
        float left  = distInches(leftDist);
        float right = distInches(rightDist);
        float front = distInches(frontDist);
        float back  = distInches(backDist);

        lemlib::Pose currentPose = chassis.getPose();

        // ── POSITION CORRECTION ──────────────────────
        // Only correct when close enough to trust the reading
        bool leftValid  = left  < 60.0f;
        bool rightValid = right < 60.0f;
        bool frontValid = front < 60.0f;
        bool backValid  = back  < 60.0f;

        float newX = currentPose.x;
        float newY = currentPose.y;
        float newTheta = currentPose.theta;

        if (leftValid)  newX = left;
        if (backValid)  newY = back;

        // ── ANGULAR CORRECTION ───────────────────────
        // Use left+right pair if both walls are visible
        if (leftValid && rightValid) {
            float angleError = getAngleFromLR();
            if (fabs(angleError) > 1.5f) // deadzone to prevent jitter
                newTheta += angleError;
        }
        // Fall back to front+back pair
        else if (frontValid && backValid) {
            float angleError = getAngleFromFB();
            if (fabs(angleError) > 1.5f)
                newTheta += angleError;
        }

        chassis.setPose(newX, newY, newTheta);

        pros::delay(20);
    }
}

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

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */



 



void autonomous() {
    // Start sensor correction in background
    pros::Task sensorTask(sensorCorrectionFn);

    // Your normal auto runs here, sensors correct in background
    chassis.moveToPoint(24, 24, 2000);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(48, 48, 2000);
    // etc...

    // Kill the task when auto is done
    sensorTask.remove();
}








/**
 * Runs in driver control
 */
float cubicDrive(float input, float scaling = 1.0f) {
    const float maxInput = 127.0f;
    return scaling * (input * input * input) / (maxInput * maxInput);
}

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

        pros::delay(10);
    }
}