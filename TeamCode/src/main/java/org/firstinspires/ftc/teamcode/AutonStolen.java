package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "gyroStolen", group = "Autonomous")
public class AutonStolen extends OpMode {
    // Figure out ticks per revolution and ticks per inch
    private static final double TICKS_PER_REV = 0;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (4.0 * Math.PI);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable


    Robot bot;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    ElapsedTime timer;

    int test = 1;

    // Variable to keep track of where in the auton the code is
    int auto = 0;

    @Override
    public void init() {
        this.bot = new Robot(this.hardwareMap, this.telemetry);
        // initialize the robot
        this.bot.initBot();
        initImu();
        timer = new ElapsedTime();
        timer.startTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("is calibrated", imu.isGyroCalibrated());
        telemetry.update();
    }

    @Override
    public void start() {
    }

    int turnAngle = 180;

    @Override
    public void loop() {
        switch (test) {
            case 1:
                /*
                 * Initialize the standard drive system variables.
                 * The init() method of the hardware class does most of the work here
                 */
                bot.init(hardwareMap);
                gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

                // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
                bot.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bot.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Send telemetry message to alert driver that we are calibrating;
                telemetry.addData(">", "Calibrating Gyro");    //
                telemetry.update();

                gyro.calibrate();

                // make sure the gyro is calibrated before continuing
                while (!isStopRequested() && gyro.isCalibrating())  {
                    sleep(50);
                    idle();
                }

                telemetry.addData(">", "Robot Ready.");    //
                telemetry.update();

                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // Wait for the game to start (Display Gyro value), and reset gyro before we move..
                while (!isStarted()) {
                    telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                    telemetry.update();
                }

                gyro.resetZAxisIntegrator();

                // Step through each leg of the path,
                // Note: Reverse movement is obtained by setting a negative distance (not speed)
                // Put a hold after each turn
//                gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
                gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
//                gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
//                gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
                gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
//                gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
                gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
//                gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
//                gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches

                telemetry.addData("Path", "Complete");
                telemetry.update();
        }

    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        bot.drive(rightSpeed, leftSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }
}