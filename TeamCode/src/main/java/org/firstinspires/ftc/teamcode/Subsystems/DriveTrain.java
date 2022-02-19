package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * The drive train subsystem. It controls the movement and driving of the bot
 *
 */
public class DriveTrain extends Subsystem {

    //region Physical Components
    private DcMotor FL, FR, BL, BR;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    //endregion

    //region Distance sensors
    public DistanceSensor distSensorRight;
    public DistanceSensor distSensorLeft;
    public DistanceSensor distSensorBack;
    //endregion

    //region Movement Stats
    private DriveTrainState driveState;
    private Direction direction;
    private double axisRightX;
    private double axisRightY;
    private double axisLeftX;
    private double axisLeftY;
    private double powerMultiplier;

    private double motorPower;
    private DistanceSensor sensorInUse;
    private double stopDist;
    //endregion

    //region Auton variables
    private ElapsedTime runtime;
    private int target;
    private int angle;
    //endregion

    //region Dependent Classes
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    //endregion

    public DriveTrain(HardwareMap hmap, Telemetry tele, boolean isAuton) {
        super(hmap, tele, isAuton);

        // Initialise dependency classes and variables
        hardwareMap = hmap;
        telemetry = tele;
        powerMultiplier = 1.0;

        // Initialise states
        driveState = DriveTrainState.TANK_TELEOP;

        // Initialize motor names
        FL = hardwareMap.get(DcMotor.class, "front_left");
        BL = hardwareMap.get(DcMotor.class, "back_left");
        FR = hardwareMap.get(DcMotor.class, "front_right");
        BR = hardwareMap.get(DcMotor.class, "back_right");

        // Set motor direction according to their orientation on the bot
        //   motors on the left side will be reversed so that their directions coorespond to
        //      the motors on the right
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        initImu();
        runtime = new ElapsedTime();

        if (isAuton) { // Set the motors to brake for ONLY auton
            driveState = DriveTrainState.SENSORDRIVE;

            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            distSensorRight = new DistanceSensor(hmap, tele, DistanceSensor.SensorName.RIGHT);
            distSensorLeft = new DistanceSensor(hmap, tele, DistanceSensor.SensorName.LEFT);
            distSensorBack = new DistanceSensor(hmap, tele, DistanceSensor.SensorName.BACK);
        }

        telemetry.addData("Drive Train", "initialized");
    }

    @Override
    public void startAuton() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //region Auton Stuff
    @Override
    public void updateState() {
        switch (driveState) {
            case ENCODERDRIVE:
                if (Math.abs(BR.getCurrentPosition()) > target) {
                    runtime.reset();
                    driveState = DriveTrainState.IDLE;
                }

                FL.setPower(motorPower);
                BL.setPower(motorPower);
                FR.setPower(motorPower);
                BR.setPower(motorPower);
                break;

            case SENSORDRIVE:
                // If the sensor reads the stop distance return true
                //    if it reads 5 inches within the stop distance, set the motor power to 6 times
                //    less than the entered power
                double distance = getSensorDistance(sensorInUse);
                if (Math.abs(distance - stopDist) < 1) {
                    stop();
                    driveState = DriveTrainState.IDLE;
                } else {
                    if (Math.abs(distance - stopDist) <= 15) {
                        this.motorPower = 0.1;
                    }

                    this.motorPower *= Math.signum(distance - stopDist);

                    if (sensorInUse == distSensorBack) {
                        FL.setPower(motorPower);
                        BL.setPower(motorPower);
                        FR.setPower(motorPower);
                        BR.setPower(motorPower);
                    } else if (sensorInUse == distSensorLeft) {
                        FL.setPower(motorPower);
                        BL.setPower(-motorPower);
                        FR.setPower(-motorPower);
                        BR.setPower(motorPower);
                    } else {
                        FL.setPower(-motorPower);
                        BL.setPower(motorPower);
                        FR.setPower(motorPower);
                        BR.setPower(-motorPower);
                    }
                }

                telemetry.addData("distance", distance);
                telemetry.addData("stop dist", stopDist);
                telemetry.addData("motor power", this.FL.getPower() + " " + FR.getPower() + " " + BL.getPower() + " " + BR.getPower() + " ");
                break;

            case ROTATE:
                // get the current heading of the bot (an angle from -180 to 180)
                float currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                // to convert to a 0-360 scale, if the current heading is negative add
                //    360 to it
                currHeading = currHeading < 0 ? 360 + currHeading : currHeading;

                // difference between target and current heading
                double difference = angle - currHeading;
                telemetry.addData("Difference is ", difference);

                // If the bot is within a 30 degree threshold of the target, slow it down to 25% of the desired speed to prevent overshooting
                if (Math.abs(difference) <= 30) {
                    FR.setPower(-motorPower / 10);
                    BR.setPower(-motorPower / 10);
                    FL.setPower(motorPower / 10);
                    BL.setPower(motorPower / 10);
                } else { // Otherwise use normal speed
                    FR.setPower(-motorPower);
                    BR.setPower(-motorPower);
                    FL.setPower(motorPower);
                    BL.setPower(motorPower);
                }

                // If the bot is within 1 degree of the target, stop the bot and return true
                if (Math.abs(difference) <= 0.5) {
                    telemetry.addLine("Stopping the bot");
                    stop();
                    driveState = DriveTrainState.IDLE;
                }
                break;

            case STOPPING:
                stop();
                setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if (runtime.milliseconds() > 500) {
                    driveState = DriveTrainState.IDLE;
                }
                break;

            case IDLE:
                runtime.reset();
                break;

            case NONE:
                break;
        }

        telemetry.addData("Drive train state", driveState);
        telemetry.addData("motor mode", FL.getMode() + " " + FR.getMode() + " " + BL.getMode() + " " + BR.getMode() + " ");
    }

    /**
     * Drives until distance sensor reads a certain distance and then returns true when there
     *
     * @param distanceSensor The distance sensor to use
     * @param distanceToStop The distance needed to stop driving
     * @param power          The power used in driving
     */
    public void driveDistanceSensor(DistanceSensor distanceSensor, double distanceToStop, double power) {
        this.sensorInUse = distanceSensor;
        this.stopDist = distanceToStop;
        this.motorPower = power;
        this.driveState = DriveTrainState.SENSORDRIVE;
    }

    // Returns the distance a given distance sensor detects
    private double getSensorDistance(DistanceSensor sensorToUse) {
        return Range.clip(sensorToUse.getDistCM(), 0.0, 200.0);
    }

    public void setTargetAndMove(int ticks, double power) {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.target = ticks;
        this.motorPower = power;
        FR.setTargetPosition(target);
        BR.setTargetPosition(target);
        FL.setTargetPosition(target);
        BL.setTargetPosition(target);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveState = DriveTrainState.ENCODERDRIVE;
    }

    // Adjusts the heading of the bot using gyroscope, degree amount to turn and motor power
    public void adjustHeading(int degrees, double power) {
        this.angle = degrees;
        this.motorPower = power;
        driveState = DriveTrainState.ROTATE;
    }
    //endregion

    //region TeleOp Stuff
    @Override
    public void updateTeleOpState(GamePadEx GP1, GamePadEx GP2) {
        if (FL.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set the joystick axis values
        axisRightX = GP1.getAxis(GamePadEx.ControllerAxis.RIGHT_X) * powerMultiplier;
        axisRightY = GP1.getAxis(GamePadEx.ControllerAxis.RIGHT_Y) * powerMultiplier;
        axisLeftX = GP1.getAxis(GamePadEx.ControllerAxis.LEFT_X) * powerMultiplier;
        axisLeftY = GP1.getAxis(GamePadEx.ControllerAxis.LEFT_Y) * powerMultiplier;
        double leftTrig = GP1.getAxis(GamePadEx.ControllerAxis.LTRIGGER) * powerMultiplier;
        double rightTrig = GP1.getAxis(GamePadEx.ControllerAxis.RTRIGGER) * powerMultiplier;
        telemetry.addData("Right X", axisRightX);
        telemetry.addData("Right Y", axisRightY);
        telemetry.addData("Left X", axisLeftX);
        telemetry.addData("Left Y", axisLeftY);

        // Perform actions based on the current state
        switch (driveState) {
            case TANK_TELEOP:
                telemetry.addLine("TANK_TELEOP");
                if (GP1.getControl(GamePadEx.ControllerButton.LTRIGGER)) { // Strafe left
                    BR.setPower(leftTrig);
                    FR.setPower(-leftTrig);
                    BL.setPower(-leftTrig);
                    FL.setPower(leftTrig);
                } else if (GP1.getControl(GamePadEx.ControllerButton.RTRIGGER)) {// Strafe right
                    BR.setPower(-rightTrig);
                    FR.setPower(rightTrig);
                    BL.setPower(rightTrig);
                    FL.setPower(-rightTrig);
                } else { // Not strafing
                    BR.setPower(axisRightY);
                    FR.setPower(axisRightY);
                    BL.setPower(axisLeftY);
                    FL.setPower(axisLeftY);
                }
                break;

            case FIELD_CENTRIC_TELEOP:
                telemetry.addLine("FIELD_CENTRIC_TELEOP");
                double orientation = Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                double sin = Math.sin(orientation);
                double cos = Math.cos(orientation);
                // forward = y
                // strafe = x
                // clockwise rotation = rx

                double temp = axisLeftY * cos - axisLeftX * sin;
                axisLeftX = axisLeftY * sin + axisLeftX * cos;
                axisLeftY = temp;

                // This denominator scales the values outside of range [1,-1]
                double denominator = Math.max(Math.abs(axisLeftY) + Math.abs(axisLeftX) + Math.abs(axisRightX), 1);

                BR.setPower(((axisLeftY + axisLeftX - axisRightX) / denominator) * 0.75);
                FR.setPower(((axisLeftY - axisLeftX - axisRightX) / denominator) * 0.75);
                BL.setPower(((axisLeftY - axisLeftX + axisRightX) / denominator) * 0.75);
                FL.setPower(((axisLeftY + axisLeftX + axisRightX) / denominator) * 0.75);

                telemetry.addData("heading: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                break;
        }

        toggleSpeeds(GP1);
    }

    /**
     * Toggle between fast and slow speeds using the gamepad buttons A and X
     *
     * @param GP1 The gamepad to use
     */
    public void toggleSpeeds(GamePadEx GP1) {
        if (GP1.getControl(GamePadEx.ControllerButton.A)) {
            powerMultiplier = 1.0;
        } else if (GP1.getControl(GamePadEx.ControllerButton.X)) {
            powerMultiplier = 0.5;
        }
    }
    //endregion

    //region General Stuff
    /**
     * Move the motors in a given direction with a given power
     *
     * @param dir The direction to drive in
     * @param drivePower The power to drive with
     */
    public void moveMotorsWithDirection(Direction dir, double drivePower) {
        BR.setPower(drivePower * dir.BR);
        FR.setPower(drivePower * dir.FR);
        BL.setPower(drivePower * dir.BL);
        FL.setPower(drivePower * dir.FL);
    }

    /**
     * ENCODERDRIVE Auton, escapable;
     * SENSORDRIVE - Auton, escapable;
     * ROTATE - Auton, escapable;
     * STOPPING - Auton, escapable;
     * IDLE - Auton, escapable;
     * TANK_TELEOP - Reset motor encoders. Auton, escapable;
     * FIELD_CENTRIC_TELEOP - TeleOP, NOT escapable;
     * WAIT - TeleOP, NOT escapable;
     * NONE - Does nothing. Fallback;
     */
    public enum DriveTrainState {
        ENCODERDRIVE, // Auton, escapable
        SENSORDRIVE, // Auton, escapable
        ROTATE, // Auton, escapable
        STOPPING, // Auton, escapable
        IDLE, // Reset encoders. Auton, escapable
        TANK_TELEOP, // TeleOP, NOT escapable
        FIELD_CENTRIC_TELEOP, // TeleOP, NOT escapable
        NONE, // Does nothing. Fallback
    }

    public enum Direction {
        NORTH(1, 1, 1, 1), // | (/\)
        NORTHEAST(1, 0, 0, 1), // / (/\)
        EAST(1, -1, -1, 1), // - (>)
        SOUTHEAST(0, -1, -1, 0), // \ (\/)
        SOUTH(-1, -1, -1, -1), // | (\/)
        SOUTHWEST(-1, 0, 0, -1), // / (\/)
        WEST(-1, 1, 1, -1), // - (<)
        NORTHWEST(0, 1, 1, 0), // \ (/\)
        TANK_TELEOP_DRIVE(1, 1, 1, 1),
        NONE(0, 0, 0, 0); // Does nothing. Fallback or no movement

        public double BR;
        public double FR;
        public double BL;
        public double FL;

        Direction(double BR, double FR, double BL, double FL) {
            this.BR = BR;
            this.FR = FR;
            this.BL = BL;
            this.FL = FL;
        }
    }

    /**
     * Sets the state to idle.
     * The idle state is used transitioning between cases in a state machine
     */
    public void waitNext() {
        driveState = DriveTrainState.NONE;
    }

    /**
     * Sets every motor to the given mode
     */
    public void setMotorMode(DcMotor.RunMode mode) {
        BR.setMode(mode);
        FR.setMode(mode);
        BL.setMode(mode);
        FL.setMode(mode);
    }
    //endregion

    /**
     * Sets the motor power of all the drive motors to 0
     */
    @Override
    public void stop() {
        BR.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        FL.setPower(0.0);
    }

    public void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }

    /**
     * Get the current state of the bot
     */
    public DriveTrainState getState() {
        return driveState;
    }
}