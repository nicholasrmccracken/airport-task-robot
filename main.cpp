// Import FEH Proteus libraries
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <FEHUtility.h>
#include <FEHSD.h>
#include <math.h>
#include <string.h>

// Declaring and initialize constants
#define POSITIVE 1
#define NEGATIVE -1

#define MIN_LEFT_SERVO_DEGREE 500.0
#define MAX_LEFT_SERVO_DEGREE 2500.0
#define MIN_RIGHT_SERVO_DEGREE 750.0
#define MAX_RIGHT_SERVO_DEGREE 2250.0

#define HIGHEST_MOVE_ANGLE 0
#define LOWEST_MOVE_ANGLE 125
#define DROP_LUGGAGE_MOVE_ANGLE 10
#define FLIP_DOWN_MOVE_ANGLE 95
#define FLIP_UP_MOVE_ANGLE 68
#define BUTTON_MOVE_ANGLE 120
#define EXIT_PASSPORT_MOVE_ANGLE 10

#define LEVEL_ROTATE_ANGLE 96
#define HOLD_LUGGAGE_ROTATE_ANGLE 130
#define DROP_LUGGAGE_ROTATE_ANGLE 75

#define EXPECTED_COUNTS_PER_INCH 40.5
#define EXPECTED_INCHES_PER_COUNT 0.02469135802
#define EXPECTED_COUNTS_PER_DEGREE 2.23

#define DEFAULT_POWER 25
#define DEFAULT_SPEED 15
#define SLOW_SPEED 9

#define P_CONSTANT 0.75
#define I_CONSTANT 0.05
#define D_CONSTANT 0.25
#define PID_SLEEP 0.1

#define PULSE_POWER 15
#define PULSE_TIME 0.1
#define RPS_SLEEP 0.1
#define NUM_POINTS_OF_INTEREST 5
#define COORD_TOLERANCE 0.5
#define HEADING_TOLERANCE 2

#define MAX_DEGREE 360
#define MAX_X 36
#define MAX_Y 72

#define START_LIGHT_TIME_OUT 30.0
#define ALIGN_TIME_OUT 5.0
#define RPS_TIME_OUT 5.0

#define MAX_VOLTAGE 3.3
#define RED_LIGHT_THRESHOLD 0.5
#define BLUE_LIGHT_THRESHOLD 1.5

#define FUEL_LEVER_SLEEP 5.0
#define POST_MOVEMENT_SLEEP 0.25
#define SCREEN_TOUCH_SLEEP 0.1

enum color
{
    BLUE_LIGHT,
    RED_LIGHT,
};

enum levers
{
    A_LEVER,
    A1_LEVER,
    B_LEVER
};

// Declare and initialize objects
AnalogInputPin lightSensor(FEHIO::P0_0);
DigitalEncoder leftEncoder(FEHIO::P1_1);
DigitalEncoder rightEncoder(FEHIO::P1_0);
FEHMotor leftMotor(FEHMotor::Motor0, 9.0);
FEHMotor rightMotor(FEHMotor::Motor1, 9.0);
FEHServo leftServo(FEHServo::Servo1);
FEHServo rightServo(FEHServo::Servo0);
DigitalInputPin backLeftBump(FEHIO::P2_1);
DigitalInputPin backRightBump(FEHIO::P2_0);
DigitalInputPin frontLeftBump(FEHIO::P2_3);
DigitalInputPin frontRightBump(FEHIO::P2_2);

class Point
{
public:
    float x;
    float y;
};

// Declare function prototypes
void drive(float speed, float inches, int direction);
void align(DigitalInputPin leftBump, DigitalInputPin rightBump, float speed, int direction);
void pivot(FEHMotor motor, DigitalEncoder encoder, int power, int degrees, int direction);
void rotateCounterclockwise(int power, int degrees, int direction);
float adjustPID(DigitalEncoder encoder, float expectedSpeed, float *oldCounts, float *oldTime, float *oldError, float *errorSum, float *oldPower);
void writeRPSPoints();
void readRPSPoints(Point points[]);
void logRPSPoint(char pointName[], float expectedX, float expectedY);
void pulseForward(int power, float seconds);
void pulseCounterclockwise(int power, float seconds);
void adjustXCoord(int power, float desiredXCoord);
void adjustYCoord(int power, float desiredYCoord);
void adjustHeading(float heading);
void initializeBlade();
void moveBlade(int degrees);
void rotateBlade(int degrees);
int determineColor(float voltage);
float scanForLight(DigitalInputPin leftBump, DigitalInputPin rightBump, float speed, int direction);
void completeRun();


/**
 * Moves the robot straight forward, a distance specified by {@code inches}.
 *
 * @param speed
 *            the speed at which the motors drive, in inches per second
 * @param inches
 *            the distance for the robot to move, in inches
 * @param direction
 *            the direction for the robot to move, with possible values of {@code POSITIVE} or {@code NEGATIVE}
 */
void drive(float speed, float inches, int direction)
{
    float time = TimeNow();
    float leftCounts = 0, leftTime = time, leftError = 0, leftErrorSum = 0, leftPower = 0;
    float rightCounts = 0, rightTime = time, rightError = 0, rightErrorSum = 0, rightPower = 0;

    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    float counts = inches * EXPECTED_COUNTS_PER_INCH;

    // While the average of the left and right encoder is less than counts, keep running motors
    while ((leftEncoder.Counts() + rightEncoder.Counts()) / 2. < counts)
    {
        Sleep(PID_SLEEP);
        leftMotor.SetPercent(direction * adjustPID(leftEncoder, speed, &leftCounts, &leftTime, &leftError, &leftErrorSum, &leftPower));
        rightMotor.SetPercent(direction * adjustPID(rightEncoder, speed, &rightCounts, &rightTime, &rightError, &rightErrorSum, &rightPower));
    }
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(POST_MOVEMENT_SLEEP);
}

/**
 * Aligns the robot straight by backing into a wall.
 *
 * @param leftBump
 *            the bump switch to make contact with the wall positioned on the left side of the chassis
 * @param rightBump
 *            the bump switch to make contact with the wall positioned on the right side of the chassis
 * @param speed
 *            the speed at which the motors drive, in inches per second
 * @param direction
 *            the direction for the robot to move, with possible values of {@code POSITIVE} or {@code NEGATIVE}
 */
void align(DigitalInputPin leftBump, DigitalInputPin rightBump, float speed, int direction)
{
    float time = TimeNow();
    float leftCounts = 0, leftTime = time, leftError = 0, leftErrorSum = 0, leftPower = 0;
    float rightCounts = 0, rightTime = time, rightError = 0, rightErrorSum = 0, rightPower = 0;

    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    // Runs until both microsensors are pressed
    while ((leftBump.Value() || rightBump.Value()) && TimeNow() < time + ALIGN_TIME_OUT)
    {
        // Half the power of microsensor if it is currently pressed
        Sleep(PID_SLEEP);
        if (!leftBump.Value())
        {
            leftMotor.SetPercent(direction * adjustPID(leftEncoder, speed / 2, &leftCounts, &leftTime, &leftError, &leftErrorSum, &leftPower));
            rightMotor.SetPercent(direction * adjustPID(rightEncoder, speed, &rightCounts, &rightTime, &rightError, &rightErrorSum, &rightPower));
        }
        else if (!rightBump.Value())
        {
            leftMotor.SetPercent(direction * adjustPID(leftEncoder, speed, &leftCounts, &leftTime, &leftError, &leftErrorSum, &leftPower));
            rightMotor.SetPercent(direction * adjustPID(rightEncoder, speed / 2, &rightCounts, &rightTime, &rightError, &rightErrorSum, &rightPower));
        }
        else
        {
            leftMotor.SetPercent(direction * adjustPID(leftEncoder, speed, &leftCounts, &leftTime, &leftError, &leftErrorSum, &leftPower));
            rightMotor.SetPercent(direction * adjustPID(rightEncoder, speed, &rightCounts, &rightTime, &rightError, &rightErrorSum, &rightPower));
        }
    }
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(POST_MOVEMENT_SLEEP);
}

/**
 * Rotates the robot a distance as indicated by {@code degrees} and a direction indicated by {@code direction}.
 *
 * @param power
 *            the power at which the motors drive, between 0 and {@code DEFAULT_POWER}
 * @param degrees
 *            the distance for the robot to move, in degrees, between 0 {@code MAX_DEGREE}
 * @param direction
 *            the direction for the robot to turn, with possible values of {@code POSITIVE} or {@code NEGATIVE}
 */
void rotateCounterclockwise(int power, int degrees, int direction)
{
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    float counts = degrees * EXPECTED_COUNTS_PER_DEGREE;

    leftMotor.SetPercent(-power * direction);
    rightMotor.SetPercent(power * direction);

    // While the average of the left and right encoder is less than counts, keep running motors
    while ((leftEncoder.Counts() + rightEncoder.Counts()) / 2. < counts) {}

    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(POST_MOVEMENT_SLEEP);
}

/**
 * Pivots the robot a distance as indicated by {@code degrees} and a direction indicated by {@code direction}.
 *
 * @param motor
 *            the motor which will drive
 * @param encoder
 *            the shaft encoder corresponding to the motor which will drive\
 * @param power
 *            the power at which the motors drive, between 0 and {@code DEFAULT_POWER}
 * @param degrees
 *            the distance for the robot to move, in degrees, between 0 {@code MAX_DEGREE}
 * @param direction
 *            the direction for the robot to turn, with possible values of {@code POSITIVE} or {@code NEGATIVE}
 */
void pivot(FEHMotor motor, DigitalEncoder encoder, int power, int degrees, int direction)
{
    encoder.ResetCounts();

    float counts = degrees * EXPECTED_COUNTS_PER_DEGREE * 2;

    motor.SetPercent(power * direction);

    // While the average of the left and right encoder is less than counts, keep running motors
    while (encoder.Counts() < counts) {}

    motor.Stop();
    Sleep(POST_MOVEMENT_SLEEP);
}

/**
 * Updates the power value of a passed in motor as indicated by {@code encoder} to account for error between the {@code expectedSpeed} and actual
 * speed as indicated by the parameters {@code *OldCounts}, {@code *oldTime}, {@code *oldError}, {@code *errorSum}, and {@code *oldPower}.
 *
 * @param encoder
 *            the encoder associated with the motor whose power is being updated
 * @param expectedSpeed
 *            the expected speed at which the motor drives, in inches per second
 * @param oldCounts
 *            the counts of the encoder from the previous adjustPID call
 * @param oldTime
 *            the time of the previous adjustPID call, in seconds
 * @param oldError
 *            the error between the expected and actual speeds of the previous adjustPID call, in inches per second
 * @param errorSum
 *            the summation of all errors between the expected and actual speeds of all adjustPID calls in the current drive call, in inches per second
 * @param oldPower
 *            the power value returned by the previous adjustPID call
 *
 * @return a float value used to set the power of a motor as indicated by {@code encoder} with respect to the error between the {@code expectedSpeed} and actual speed
 */
float adjustPID(DigitalEncoder encoder, float expectedSpeed, float *oldCounts, float *oldTime, float *oldError, float *errorSum, float *oldPower)
{
    float newCounts = encoder.Counts();
    float deltaCounts = newCounts - *oldCounts;
    *oldCounts = newCounts;

    float newTime = TimeNow();
    float deltaTime = newTime - *oldTime;
    *oldTime = newTime;

    float actualSpeed = deltaCounts * EXPECTED_INCHES_PER_COUNT / deltaTime;

    float newError = expectedSpeed - actualSpeed;
    float deltaError = newError - *oldError;
    *oldError = newError;
    *errorSum += newError;

    float PTerm = newError * P_CONSTANT;
    float ITerm = *errorSum * I_CONSTANT;
    float DTerm = deltaError * D_CONSTANT;

    float newPower = *oldPower + PTerm + ITerm + DTerm;
    *oldPower = newPower;
    return newPower;
}

/**
 * Pulse forward for a short time as indicated by {@ seconds}.
 *
 * @param power
 *            the power at which the motors drive, between -{@code MAX_POWER} and {@code MAX_POWER}
 * @param seconds
 *            the length of time the motors drive
 */
void pulseForward(int power, float seconds)
{
    rightMotor.SetPercent(power);
    leftMotor.SetPercent(power);

    Sleep(seconds);

    rightMotor.Stop();
    leftMotor.Stop();
}

/**
 * Pulse counterclockwise for a short time as indicated by {@ seconds}.
 *
 * @param power
 *            the power at which the motors drive, between -{@code MAX_POWER} and {@code MAX_POWER}
 * @param seconds
 *            the time the motors drive, greater than 0
 */
void pulseCounterclockwise(int power, float seconds)
{
    rightMotor.SetPercent(power);
    leftMotor.SetPercent(-power);

    Sleep(seconds);

    rightMotor.Stop();
    leftMotor.Stop();
}

/**
 * Initialize all RPS points by writing their coordinates to a file.
 */
void writeRPSPoints()
{
    // Declare variables
    float touchX, touchY;
    char points[NUM_POINTS_OF_INTEREST][20] = {"A1_LEVER", "A_LEVER", "B_LEVER", "POST_RAMP", "LIGHT"};

    // Open SD file for writing
    FEHFile *fptr = SD.FOpen("RPS_Points.txt", "w");

    Sleep(SCREEN_TOUCH_SLEEP);
    LCD.Clear();
    LCD.ClearBuffer();

    // Write initial screen info
    LCD.WriteRC("X Position:", 11, 0);
    LCD.WriteRC("Y Position:", 12, 0);
    LCD.WriteRC("   Heading:", 13, 0);

    // Step through each path point to record position and heading
    for (int i = 0; i < NUM_POINTS_OF_INTEREST; i++)
    {
        // Write point letter
        LCD.WriteRC("Set Current Point: ", 9, 0);
        LCD.WriteRC(points[i], 9, 19);

        // Wait for touchscreen to be pressed and display RPS data
        while (!LCD.Touch(&touchX, &touchY))
        {
            LCD.WriteRC(RPS.X(), 11, 12);       // update the x coordinate
            LCD.WriteRC(RPS.Y(), 12, 12);       // update the y coordinate
            LCD.WriteRC(RPS.Heading(), 13, 12); // update the heading

            Sleep(SCREEN_TOUCH_SLEEP); // wait for 100ms to avoid updating the screen too quickly
        }
        while (LCD.Touch(&touchX, &touchY)) {}
        LCD.ClearBuffer();

        // Print RPS data for this path point to file
        SD.FPrintf(fptr, "%f %f\n", RPS.X(), RPS.Y());
    }

    // Close SD file
    SD.FClose(fptr);
    LCD.Clear();
}

/**
 * Store the coordinates of all points of interest by reading their coordinates from a file.
 * 
 * @param points
 *            an empty array used to store the coordinates of all points of interest
 */
void readRPSPoints(Point points[])
{
    FEHFile *fptr = SD.FOpen("RPS_Points.txt", "r");

    for (int i = 0; i < NUM_POINTS_OF_INTEREST; i++)
    {
        SD.FScanf(fptr, "%f%f", points[i].x, points[i].y);
    }
    SD.FClose(fptr);
}

/**
 * Log the current coordinates of the robot and the expected coordinates of one point of interest in a file.
 * 
 * @param pointName
 *            an array of strings used to refer to each point of interest
 * @param expectedX
 *            x coordinate of the point of interest
 * @param expectedY
 *            y coordinate of the point of interest
 */
void logRPSPoint(char pointName[], float expectedX, float expectedY)
{
    FEHFile *fptr = SD.FOpen("Run_Data.txt", "w");

    SD.FPrintf(fptr, "Expected %s Position: %f %f\n", pointName, expectedX, expectedY);
    SD.FPrintf(fptr, "Actual %s Position:   %f %f\n\n", pointName, RPS.X(), RPS.Y());

    SD.FClose(fptr);
}

/**
 * Use RPS to move to the desired x coordinate based on the QR code's current position as indicated by {@code orientation} which
 * is positive when the direction the robot is currently facing is the positive x direction with respect to the course, and vise
 * versa for the negative direction.
 *
 * @param orientation
 *            the current orientation of the qr code, with possible values of {@code POSITIVE} or {@code NEGATIVE}
 * @param desiredXCoord
 *            the desired y coordinate, between 0 and {@code MAX_X}
 */
void adjustXCoord(int orientation, float desiredXCoord)
{
    int power = PULSE_POWER;
    if (orientation == NEGATIVE)
    {
        power = -PULSE_POWER;
    }
    float currentXCoord = RPS.X();
    float time = TimeNow();

    while ((currentXCoord >= 0 && currentXCoord <= MAX_X && (currentXCoord < desiredXCoord - COORD_TOLERANCE || currentXCoord > desiredXCoord + COORD_TOLERANCE)) && TimeNow() < time + RPS_TIME_OUT)
    {
        if (currentXCoord > desiredXCoord)
        {
            pulseForward(-power, PULSE_TIME);
        }
        else if (currentXCoord < desiredXCoord)
        {
            pulseForward(power, PULSE_TIME);
        }
        Sleep(RPS_SLEEP);
        currentXCoord = RPS.X();
    }
}

/**
 * Use RPS to move to the desired y coordinate based on the QR code's current position as indicated by {@code orientation} which
 * is positive when the direction the robot is currently facing is the positive y direction with respect to the course, and vise
 * versa for the negative direction.
 *
 * @param orientation
 *            the current orientation of the qr code, with possible values of {@code POSITIVE} or {@code NEGATIVE}
 * @param desiredYCoord
 *            the desired y coordinate, between 0 and {@code MAX_Y}
 */
void adjustYCoord(int orientation, float desiredYCoord)
{
    int power = PULSE_POWER;
    if (orientation == NEGATIVE)
    {
        power = -PULSE_POWER;
    }
    float currentYCoord = RPS.Y();
    float time = TimeNow();

    while ((currentYCoord >= 0 && currentYCoord <= MAX_Y && (currentYCoord < desiredYCoord - COORD_TOLERANCE || currentYCoord > desiredYCoord + COORD_TOLERANCE)) && TimeNow() < time + RPS_TIME_OUT)
    {
        if (currentYCoord > desiredYCoord)
        {
            pulseForward(-power, PULSE_TIME);
        }
        else if (currentYCoord < desiredYCoord)
        {
            pulseForward(power, PULSE_TIME);
        }
        Sleep(RPS_SLEEP);
        currentYCoord = RPS.Y();
    }
}

/**
 * Use RPS to move to the desired heading angle.
 *
 * @param desiredHeading
 *            the desired heading angle, between 0 and {@code MAX_DEGREE}
 */
void adjustHeading(float desiredHeading)
{
    float actualHeading = RPS.Heading(), counterClockwiseDistance = 0, clockwiseDistance = 0;
    float time = TimeNow();

    while ((actualHeading >= 0 && actualHeading <= MAX_DEGREE &&
            fabs(actualHeading - desiredHeading) > HEADING_TOLERANCE && MAX_DEGREE - fabs(actualHeading - desiredHeading) > HEADING_TOLERANCE) &&
           TimeNow() < time + RPS_TIME_OUT)
    {
        if (actualHeading - desiredHeading > desiredHeading - actualHeading)
        {
            counterClockwiseDistance = MAX_DEGREE - fabs(actualHeading - desiredHeading);
            clockwiseDistance = fabs(actualHeading - desiredHeading);
        }
        else
        {
            counterClockwiseDistance = fabs(actualHeading - desiredHeading);
            clockwiseDistance = MAX_DEGREE - fabs(actualHeading - desiredHeading);
        }

        if (counterClockwiseDistance < clockwiseDistance)
        {
            pulseCounterclockwise(PULSE_POWER, PULSE_TIME);
        }
        else
        {
            pulseCounterclockwise(-PULSE_POWER, PULSE_TIME);
        }
        Sleep(RPS_SLEEP);
        actualHeading = RPS.Heading();
    }
}

/**
 * Initializes both servos min and max and sets the blade to the proper position to begin the run.
 */
void initializeBlade()
{
    leftServo.SetMin(MIN_LEFT_SERVO_DEGREE);
    leftServo.SetMax(MAX_LEFT_SERVO_DEGREE);
    rightServo.SetMin(MIN_RIGHT_SERVO_DEGREE);
    rightServo.SetMax(MAX_RIGHT_SERVO_DEGREE);

    moveBlade(HIGHEST_MOVE_ANGLE);
    rotateBlade(HOLD_LUGGAGE_ROTATE_ANGLE);
}

/**
 * Moves the bulldozer blade as indicated by {@code degrees} and correspondingly adjusts the angle of the blade.
 *
 * @param degrees
 *            the angle value for the bulldozer blade to move
 */
void moveBlade(int degrees)
{
    leftServo.SetDegree(degrees);
}

/**
 * Turns the bulldozer blade as indicated by {@code degrees}.
 *
 * @param degrees
 *            the angle value for the bulldozer blade to turn
 */
void rotateBlade(int degrees)
{
    rightServo.SetDegree(degrees);
}

/**
 * Determines if the sensed light is blue or red as indicated by {@code voltage}.
 *
 * @param voltage
 *            the voltage value returned by the CdS cell, between 0 and {@code MAX_VOLTAGE}
 *
 * @return an integer corresponding to which light color is displayed as defined by {@code enum color}
 */
int determineColor(float voltage)
{
    int lightColor = BLUE_LIGHT;

    if (voltage < RED_LIGHT_THRESHOLD)
    {
        lightColor = RED_LIGHT;
        LCD.Clear(RED);
    }
    else
    {
        LCD.Clear(BLUE);
    }
    return lightColor;
}

/**
 * Function that finds the minimum voltage value detected by the CdS cell, and returns it to determine the color of the light.
 *
 * @param leftBump
 *            left microswitch used to detect when the robot is aligned with a wall
 * @param rightBump
 *            right microswitch used to detect when the robot is aligned with a wall
 * @param speed
 *            the speed at which the robot will travel
 * @param direction
 *            1 for forwards, -1 for backwards
 * @return minimum voltage detected
 */
float scanForLight(DigitalInputPin leftBump, DigitalInputPin rightBump, float speed, int direction)
{
    float time = TimeNow();
    float leftCounts = 0, leftTime = time, leftError = 0, leftErrorSum = 0, leftPower = 0;
    float rightCounts = 0, rightTime = time, rightError = 0, rightErrorSum = 0, rightPower = 0;
    float currentVoltage = lightSensor.Value();
    float minVoltage = currentVoltage;

    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    // Runs until both microsensors are pressed
    while ((leftBump.Value() || rightBump.Value()) && TimeNow() < time + ALIGN_TIME_OUT)
    {
        // Half the power of microsensor if it is currently pressed
        if (minVoltage > currentVoltage)
        {
            minVoltage = currentVoltage;
        }
        Sleep(PID_SLEEP);
        if (!leftBump.Value())
        {
            leftMotor.SetPercent(direction * adjustPID(leftEncoder, speed / 2, &leftCounts, &leftTime, &leftError, &leftErrorSum, &leftPower));
            rightMotor.SetPercent(direction * adjustPID(rightEncoder, speed, &rightCounts, &rightTime, &rightError, &rightErrorSum, &rightPower));
        }
        else if (!rightBump.Value())
        {
            leftMotor.SetPercent(direction * adjustPID(leftEncoder, speed, &leftCounts, &leftTime, &leftError, &leftErrorSum, &leftPower));
            rightMotor.SetPercent(direction * adjustPID(rightEncoder, speed / 2, &rightCounts, &rightTime, &rightError, &rightErrorSum, &rightPower));
        }
        else
        {
            leftMotor.SetPercent(direction * adjustPID(leftEncoder, speed, &leftCounts, &leftTime, &leftError, &leftErrorSum, &leftPower));
            rightMotor.SetPercent(direction * adjustPID(rightEncoder, speed, &rightCounts, &rightTime, &rightError, &rightErrorSum, &rightPower));
        }
        currentVoltage = lightSensor.Value();
    }
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(POST_MOVEMENT_SLEEP);

    return minVoltage;
}

/**
 * Fuction that contains all methods necessary for complete course run.
 */
void completeRun()
{
    initializeBlade();

    RPS.InitializeTouchMenu();

    float xTrash, yTrash;
    LCD.Clear(RED);
    while (!LCD.Touch(&xTrash, &yTrash));
    while (LCD.Touch(&xTrash, &yTrash));
    Sleep(SCREEN_TOUCH_SLEEP);
    LCD.Clear(GREEN);

    float time = TimeNow();

    // Robot initiates with start light
    while (lightSensor.Value() > 1 && TimeNow() < time + START_LIGHT_TIME_OUT);

    // Get the correct lever at start to ensure no deadzone interference
    int lever = RPS.GetCorrectLever();

    // Navigate to face upper luggage dropoff
    drive(DEFAULT_SPEED, 12, POSITIVE);
    rotateCounterclockwise(DEFAULT_POWER, 45, NEGATIVE);
    adjustHeading(180);
    drive(DEFAULT_SPEED, 4, POSITIVE);

    // Drop luggage into box then return it to default position
    moveBlade(DROP_LUGGAGE_MOVE_ANGLE);
    rotateBlade(DROP_LUGGAGE_ROTATE_ANGLE);
    Sleep(1.0);
    rotateBlade(LEVEL_ROTATE_ANGLE);

    // Drive from luggage task to the upper level
    drive(DEFAULT_SPEED, 1.5, NEGATIVE);
    rotateCounterclockwise(DEFAULT_POWER, 90, NEGATIVE);
    adjustHeading(90);
    
    // Navigate to face correct lever
    if (lever == A_LEVER) {
        drive(DEFAULT_SPEED, 1, NEGATIVE);
        pivot(leftMotor, leftEncoder, DEFAULT_POWER, 90, POSITIVE);
    } else if (lever == A1_LEVER) {
        drive(DEFAULT_SPEED, 4.5, NEGATIVE);
        pivot(leftMotor, leftEncoder, DEFAULT_POWER, 90, POSITIVE);
    } else {
        align(backLeftBump, backRightBump, SLOW_SPEED, NEGATIVE);
        pivot(leftMotor, leftEncoder, DEFAULT_POWER, 90, POSITIVE);
    }
    adjustHeading(0);
    drive(DEFAULT_SPEED, 1, POSITIVE);

    // Flip lever down, wait five seconds, then flip lever up
    moveBlade(FLIP_DOWN_MOVE_ANGLE);
    drive(DEFAULT_SPEED, 1.5, NEGATIVE);
    adjustHeading(0);
    moveBlade(LOWEST_MOVE_ANGLE);
    drive(DEFAULT_SPEED, 1.5, POSITIVE);
    Sleep(FUEL_LEVER_SLEEP);
    moveBlade(FLIP_UP_MOVE_ANGLE);
    Sleep(1.0);
    moveBlade(LOWEST_MOVE_ANGLE);

    // Navigate up ramp
    drive(DEFAULT_SPEED, 1, NEGATIVE);
    pivot(rightMotor, rightEncoder, DEFAULT_POWER, 90, NEGATIVE);
    adjustHeading(90);
    align(backLeftBump, backRightBump, SLOW_SPEED, NEGATIVE);
    drive(DEFAULT_SPEED, 1, POSITIVE);
    rotateCounterclockwise(DEFAULT_POWER, 90, POSITIVE);
    adjustHeading(180);
    drive(DEFAULT_SPEED, 17, POSITIVE);

    // Align immediately after driving up ramp
    rotateCounterclockwise(DEFAULT_POWER, 90, NEGATIVE);
    adjustHeading(90);
    align(backLeftBump, backRightBump, SLOW_SPEED, NEGATIVE);
    drive(DEFAULT_SPEED, 7.5, POSITIVE);

    // Align against wall then find light
    rotateCounterclockwise(DEFAULT_POWER, 90, POSITIVE);
    adjustHeading(180);
    drive(DEFAULT_SPEED, 8, NEGATIVE);
    drive(DEFAULT_SPEED, 12.5, POSITIVE);

    // Back into wall
    rotateCounterclockwise(DEFAULT_POWER, 45, POSITIVE);
    adjustHeading(225);

    // Determine light color
    float lightVoltage = scanForLight(frontLeftBump, frontRightBump, SLOW_SPEED, POSITIVE);
    int light = determineColor(lightVoltage);

    // Move to passport stamp
    drive(DEFAULT_SPEED, 6.5, NEGATIVE);
    moveBlade(LOWEST_MOVE_ANGLE);
    rotateBlade(LEVEL_ROTATE_ANGLE);
    rotateCounterclockwise(DEFAULT_POWER, 225, POSITIVE);
    adjustHeading(90);
    drive(SLOW_SPEED, 1, POSITIVE);

    // Flip passport stamp past 90 degrees then lower blade slightly and back up
    moveBlade(HIGHEST_MOVE_ANGLE);
    Sleep(1.5);
    moveBlade(EXIT_PASSPORT_MOVE_ANGLE);

    // Navigate to face button corresponding to light color
    if (light == BLUE_LIGHT)
    {
        drive(DEFAULT_SPEED, 1, NEGATIVE);
        moveBlade(HIGHEST_MOVE_ANGLE);
        rotateCounterclockwise(DEFAULT_POWER, 90, NEGATIVE);
        adjustHeading(0);
        drive(DEFAULT_SPEED, 7.5, NEGATIVE);
        adjustHeading(0);
        drive(DEFAULT_SPEED, 14, POSITIVE);
        rotateCounterclockwise(DEFAULT_POWER, 90, POSITIVE);
        adjustHeading(90);
    }
    else
    {
        pivot(rightMotor, rightEncoder, DEFAULT_POWER, 90, POSITIVE);
        adjustHeading(180);
        moveBlade(HIGHEST_MOVE_ANGLE);
        drive(DEFAULT_SPEED, 7.5, POSITIVE);
        adjustHeading(180);
        drive(DEFAULT_SPEED, 14, NEGATIVE);
        rotateCounterclockwise(DEFAULT_POWER, 90, NEGATIVE);
        adjustHeading(90);
    }
    // Navigate to hit final button
    align(frontLeftBump, frontRightBump, SLOW_SPEED, POSITIVE);
    pivot(rightMotor, rightEncoder, DEFAULT_POWER, 90, NEGATIVE);
    adjustHeading(0);
    drive(DEFAULT_SPEED, 30, POSITIVE);

    // Extra movement if robot misses final button
    adjustHeading(0);
    rotateCounterclockwise(DEFAULT_SPEED, 45, POSITIVE);
    drive(DEFAULT_SPEED, 5, NEGATIVE);
    rotateCounterclockwise(DEFAULT_SPEED, 45, POSITIVE);
    adjustHeading(45);
    align(frontLeftBump, frontRightBump, SLOW_SPEED, POSITIVE);
}

int main(void)
{
    completeRun();
}
