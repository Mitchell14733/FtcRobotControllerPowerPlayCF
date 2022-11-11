/* Driving with mech wheels
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*********************************************/

@TeleOp(name="TeleOpPowerPLay", group="Linear Opmode")
//@Disabled

public class TeleOpPowerPLay extends LinearOpMode {

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    private DcMotor left_front = null;
    private DcMotor right_front = null;
    private DcMotor right_rear = null;
    private DcMotor left_rear = null;
    private DcMotor slide_motor = null;
    private Servo Back;
    private Servo Front;
    private DigitalChannel touch;
    private double left_front_power;
    private double right_front_power;
    private double left_rear_power;
    private double right_rear_power;

    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftFrontTarget = 0;
    private int leftRearTarget = 0;
    private int rightFrontTarget = 0;
    private int rightRearTarget = 0;

    @Override
    public void runOpMode() {
//        telemetry.addData("Status", "Initialized");
        telemetry.update();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port.
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        double driveTurn;
        double gamepadXCoordinate;
        double gamepadYCoordinate;
        double gamepadHypot = 0;
        double gamepadRadians = 0;
        double robotRadians = 0;
        double correctedRobotRadians = 0;
        double movementRadians = 0;
        double gamepadXControl = 0;
        double gamepadYControl = 0;
        boolean collectionMode;
        boolean coneReceived;

        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_rear = hardwareMap.get(DcMotor.class, "left_rear");
        right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        slide_motor = hardwareMap.get(DcMotor.class, "slide_motor");
        Back = hardwareMap.get(Servo.class, "Back");
        Front = hardwareMap.get(Servo.class, "Front");
        touch = hardwareMap.get(DigitalChannel.class, "touch");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        left_front.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        left_rear.setDirection(DcMotor.Direction.REVERSE);
        right_rear.setDirection(DcMotor.Direction.FORWARD);
        slide_motor.setDirection(DcMotor.Direction.FORWARD);

        slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_motor.setTargetPosition(0);
        slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide_motor.setPower(0.75);
        collectionMode = false;
        coneReceived = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /* Adjust Joystick X/Y inputs by navX MXP yaw angle */
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float gyro_degrees = (angles.firstAngle) - (float)headingOffset;
            telemetry.addData("Yaw", ("%.3f"),gyro_degrees);
            telemetry.addData("Left X", ("%.3f"), gamepad1.left_stick_x);
            telemetry.addData("Right X", ("%.3f"), gamepad1.right_stick_x);
            telemetry.addData("Right Y", ("%.3f"), gamepad1.right_stick_y);
            telemetry.addData("gamepadHypot", ("%.3f"), gamepadHypot);
            telemetry.addData("gamepadDegree", ("%.3f"), gamepadRadians);
            telemetry.addData("movementDegree", ("%.3f"), movementRadians);
            telemetry.addData("gamepadXControl", ("%.3f"), gamepadXControl);
            telemetry.addData("gamepadYControl", ("%.3f"), gamepadYControl);
            telemetry.addData("RF POWER", ("%.3f"), right_front_power);
            telemetry.addData("RR POWER", ("%.3f"), right_rear_power);
            telemetry.addData("LF POWER", ("%.3f"), left_front_power);
            telemetry.addData("LR POWER", ("%.3f"), left_rear_power);
            telemetry.addData("Slide", "%7d", slide_motor.getCurrentPosition());

            driveTurn = -gamepad1.left_stick_x;
            gamepadXCoordinate = gamepad1.right_stick_x; //this simply gives our x value relative to the driver
            gamepadYCoordinate = -gamepad1.right_stick_y; //this simply gives our y value relative to the driver
            gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);

            //finds just how much power to give the robot based on how much x and y given by gamepad
            //range.clip helps us keep our power within positive 1
            // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)

            gamepadRadians = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);// - Math.PI/2; //the inverse tangent of opposite/adjacent gives us our gamepad degree

            robotRadians = (gyro_degrees * Math.PI / 180); //gives us the angle our robot is at, in radians

            movementRadians = gamepadRadians - robotRadians; //adjust the angle we need to move at by finding needed
            // movement degree based on gamepad and robot angles
            gamepadXControl = Math.cos(movementRadians) * gamepadHypot;
            //by finding the adjacent side, we can get our needed x value to power our motors
            gamepadYControl = Math.sin(movementRadians) * gamepadHypot;
            //by finding the opposite side, we can get our needed y value to power our motors

            //by multiplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will
            // not exceed 1 without any driveTurn
            //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
            //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
            right_front_power = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
            right_rear_power = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
            left_front_power = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
            left_rear_power = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
            right_front.setPower(right_front_power * DRIVE_SPEED);
            left_front.setPower(left_front_power * DRIVE_SPEED);
            right_rear.setPower(right_rear_power * DRIVE_SPEED);
            left_rear.setPower(left_rear_power * DRIVE_SPEED);

            //Declare other button functions here
            //*****************************     Gamepad 1     **************************************
            if (gamepad1.left_bumper){ //Slow down for precision
                DRIVE_SPEED = 0.25;
            }
            else{
                DRIVE_SPEED = 0.65;
            }

            //Spin 180 degrees
            //if (gamepad1.right_bumper) {
            //    float currentHeading = angles.firstAngle;
            //    turnToHeading(TURN_SPEED, currentHeading - 180);
          //  }

            //Reset Heading
           while (gamepad1.right_bumper) {
               resetHeading();
           }

            //*****************************     Gamepad 2     **************************************
            //Pick up cone
            if (gamepad2.a) {
                collectionMode = true; //waiting for cone to trigger touch sensor
                slide_motor.setTargetPosition(100); //Drop until cone is picked up
                Back.setPosition(0); //turn servos to intake mode
                Front.setPosition(1);
            }
            //Cone is collected
            if (collectionMode) {
                if (!touch.getState()) { //!touch.getState() = switch is pressed
                    collectionMode = false; //no longer in collection mode
                    coneReceived = true; //triggers the keepCone method
                    slide_motor.setTargetPosition(1400); //pop slide to the middle position
                }
            }
            if (coneReceived) {
                    keepCone(); //turns on servos if switch contact is lost
                }

            if (gamepad2.b) { //drop the cone
                coneReceived = false; //reset this variable
                Back.setPosition(1);
                Front.setPosition(0);
                sleep(350); //nobody move while we drop this!
                Back.setPosition(0.5);
                Front.setPosition(0.5);
            }
            if (gamepad2.dpad_up) {
                slide_motor.setTargetPosition(3500); //move slide to High position
            }
            if (gamepad2.dpad_right) {
                slide_motor.setTargetPosition(2500); //move slide to Medium position
            }
            if (gamepad2.dpad_down) {
                slide_motor.setTargetPosition(1550); //move slide to Low position
            }
            if (gamepad2.dpad_left) {
                slide_motor.setTargetPosition(1050); //move slide to cone pickup position/Off the ground for ground junction
            }
            telemetry.update();

        } //End of while op mode is active
    } //End of run OP Mode

    private void keepCone() {
        if (touch.getState()) {
            Back.setPosition(0);
            Front.setPosition(1);
        } else {
            Back.setPosition(0.5);
            Front.setPosition(0.5);
        }
    }

        //Constants and functions for adding automatic steering controls
        static final double COUNTS_PER_MOTOR_REV = 28.0;   // Rev Ultraplanetary HD Hex motor: 28.0
        static final double DRIVE_GEAR_REDUCTION = 20.0;     // External Gear Ratio
        static final double WHEEL_DIAMETER_INCHES = 3.78;     // 96mm Mech Wheels, For figuring circumference
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        // These constants define the desired driving/control characteristics
        // They can/should be tweaked to suit the specific robot drive train.
        static double DRIVE_SPEED = 0.75;     // Max driving speed for better distance accuracy.
        static final double TURN_SPEED = 1.0;     // Max Turn speed to limit turn rate
        static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
        // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
        // Define the Proportional control coefficient (or GAIN) for "heading control".
        // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
        // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
        // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
        static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
        static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable

        public void turnToHeading ( double maxTurnSpeed, double heading){

            // Run getSteeringCorrection() once to pre-calculate the current error
            getSteeringCorrection(heading, P_DRIVE_GAIN);

            // keep looping while we are still active, and not on heading.
            while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

                // Clip the speed to the maximum permitted value.
                turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

                // Pivot in place by applying the turning correction
                moveRobot(0, turnSpeed);

            }
            moveRobot(0, 0);
        }

        /**
         * Reset the "offset" heading back to zero
         */
        public void resetHeading () {
            // Save a new heading offset equal to the current raw heading.
            headingOffset = angles.firstAngle;
            robotHeading = 0;
        }

        public double getSteeringCorrection ( double desiredHeading, double proportionalGain){
            targetHeading = desiredHeading;  // Save for telemetry

            // Get the robot heading by applying an offset to the IMU heading
            robotHeading = angles.firstAngle - headingOffset;

            // Determine the heading current error
            headingError = targetHeading - robotHeading;

            // Normalize the error to be within +/- 180 degrees
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
            return Range.clip(headingError * proportionalGain, -1, 1);
        }

        public void moveRobot ( double drive, double turn){
            driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
            turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

            leftSpeed = drive - turn;
            rightSpeed = drive + turn;

            // Scale speeds down if either one exceeds +/- 1.0;
            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            left_front.setPower(leftSpeed);
            left_rear.setPower(leftSpeed);
            right_front.setPower(rightSpeed);
            right_rear.setPower(rightSpeed);
        }
}
