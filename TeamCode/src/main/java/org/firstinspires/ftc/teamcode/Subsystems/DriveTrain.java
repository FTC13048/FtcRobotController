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

public class DriveTrain extends Subsystem {

    //region Physical Components
    private DcMotor FL, FR, BL, BR;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;

    //Distance sensors
    public DistanceSensor distSensorRight;
    public DistanceSensor distSensorLeft;
    public DistanceSensor distSensorBack;

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

    // auton variables
    private ElapsedTime runtime;
    private int target;
    private int angle;

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
            driveState = DriveTrainState.MOVESENSOR;

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
            case MOVEENCODER:
                moveMotorsWithDirection(this.direction, 0.5);
                if (Math.abs(BR.getCurrentPosition()) > target) {
                    runtime.reset();
                    driveState = DriveTrainState.STOPPING;
                }
                break;

            case MOVESENSOR:
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

                    if(sensorInUse == distSensorBack){
                        FL.setPower(motorPower);
                        BL.setPower(motorPower);
                        FR.setPower(motorPower);
                        BR.setPower(motorPower);
                    } else if(sensorInUse == distSensorLeft){
                        FL.setPower(motorPower);
                        BL.setPower(-motorPower);
                        FR.setPower(-motorPower);
                        BR.setPower(motorPower);
                    } else{
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

            case TURN:
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
                    BR.setPower(-motorPower/ 10);
                    FL.setPower(motorPower/ 10);
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

            case WAIT:
                break;
        }

        telemetry.addData("drive state", driveState);
        telemetry.addData("motor mode", FL.getMode() + " " + FR.getMode() + " " + BL.getMode() + " " + BR.getMode() + " ");
    }

    // Drives until distance sensor reads a certain distance and then returns true when there
    public void driveDistanceSensor(DistanceSensor distanceSensor, double distanceToStop, double power) {
        this.sensorInUse = distanceSensor;
        this.stopDist = distanceToStop;
        this.motorPower = power;
        this.driveState = DriveTrainState.MOVESENSOR;
    }

    // Returns the distance a given distance sensor detects
    private double getSensorDistance(DistanceSensor sensorToUse) {
        return Range.clip(sensorToUse.getDistCM(), 0.0, 200.0);
    }

    public void setTargetAndMove(int ticks, Direction d){
        target = ticks;
        direction = d;
        FR.setTargetPosition(target);
        BR.setTargetPosition(target);
        FL.setTargetPosition(target);
        BL.setTargetPosition(target);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveState = DriveTrainState.MOVEENCODER;
    }

    // Adjusts the heading of the bot using gyroscope, degree amount to turn and motor power
    public void adjustHeading(int degrees, double power) {
        this.angle = degrees;
        this.motorPower = power;
        driveState = DriveTrainState.TURN;
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

    public void toggleSpeeds(GamePadEx GP1) {
        if (GP1.getControl(GamePadEx.ControllerButton.A)) {
            powerMultiplier = 1.0;
        } else if (GP1.getControl(GamePadEx.ControllerButton.X)) {
            powerMultiplier = 0.5;
        }
    }
    //endregion

    //region General Stuff
    public void moveMotorsWithDirection(Direction dir, double drivePower) {
        switch (dir) {
            case NORTH:
                BR.setPower(drivePower);
                FR.setPower(drivePower);
                BL.setPower(drivePower);
                FL.setPower(drivePower);

            case NORTHEAST:
                BR.setPower(drivePower);
                FR.setPower(0.0);
                BL.setPower(0.0);
                FL.setPower(drivePower);

            case EAST:
                BR.setPower(drivePower);
                FR.setPower(-drivePower);
                BL.setPower(-drivePower);
                FL.setPower(drivePower);

            case SOUTHEAST:
                BR.setPower(0.0);
                FR.setPower(-drivePower);
                BL.setPower(-drivePower);
                FL.setPower(0.0);

            case SOUTH:
                BR.setPower(-drivePower);
                FR.setPower(-drivePower);
                BL.setPower(-drivePower);
                FL.setPower(-drivePower);

            case SOUTHWEST:
                BR.setPower(-drivePower);
                FR.setPower(0.0);
                BL.setPower(0.0);
                FL.setPower(-drivePower);

            case WEST:
                BR.setPower(-drivePower);
                FR.setPower(drivePower);
                BL.setPower(drivePower);
                FL.setPower(-drivePower);

            case NORTHWEST:
                BR.setPower(0.0);
                FR.setPower(drivePower);
                BL.setPower(drivePower);
                FL.setPower(0.0);

            case TANK_TELEOP_DRIVE:
                BR.setPower(axisLeftY);
                FR.setPower(axisLeftY);
                BL.setPower(axisRightY);
                FL.setPower(axisRightY);
        }
    }

    public enum DriveTrainState {
        MOVEENCODER, // Auton, escapable
        MOVESENSOR,
        TURN, // Auton, escapable
        STOPPING, // Auton, escapable
        IDLE, // Reset encoders. Auton, escapable
        TANK_TELEOP, // TeleOP, NOT escapable
        FIELD_CENTRIC_TELEOP, // TeleOP, NOT escapable
        WAIT,
        NONE, // Does nothing. Fallback
    }

    public enum Direction {
        NORTH, // | (/\)
        NORTHEAST, // / (/\)
        EAST, // - (>)
        SOUTHEAST, // \ (\/)
        SOUTH, // | (\/)
        SOUTHWEST, // / (\/)
        WEST, // - (<)
        NORTHWEST, // \ (/\)
        TANK_TELEOP_DRIVE,
        NONE, // Does nothing. Fallback or no movement
    }

    public void waitNext(){
        driveState = DriveTrainState.WAIT;
    }

    public void setMotorMode(DcMotor.RunMode mode) {
        BR.setMode(mode);
        FR.setMode(mode);
        BL.setMode(mode);
        FL.setMode(mode);
    }
    //endregion

    // Sets the motor power of all the drive motors to 0
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

    public DriveTrainState getState(){
        return driveState;
    }
}