package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private double drivePower;
    private double axisRightX;
    private double axisRightY;
    private double axisLeftX;
    private double axisLeftY;
    //endregion

    //region Dependent Classes
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    //endregion

    public DriveTrain(HardwareMap hmap, Telemetry tele, boolean isAuton) { // REMOVE THE ISAUTON BOOLEAN
        super(hmap, tele, isAuton);

        // Initialise dependency classes
        hardwareMap = hmap;
        telemetry = tele;

        // Initialise states
        driveState = DriveTrainState.IDLE;

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

        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initImu();

        if (isAuton) { // Set the motors to brake for ONLY auton
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
    public void updateState() {

    }

    @Override
    public void updateTeleOpState(GamePadEx DrivingGP, GamePadEx OtherGP) {
        if (driveState == DriveTrainState.IDLE) { // CHANGE THIS, ONLY FOR TESTING
            driveState = DriveTrainState.TANK_TELEOP;
            direction = Direction.TANK_TELEOP_DRIVE;
        }

        // Set the joystick axis values
        axisRightX = DrivingGP.getAxis(GamePadEx.ControllerAxis.RIGHT_X);
        axisRightY = DrivingGP.getAxis(GamePadEx.ControllerAxis.RIGHT_Y);
        axisLeftX = DrivingGP.getAxis(GamePadEx.ControllerAxis.LEFT_X);
        axisLeftY = DrivingGP.getAxis(GamePadEx.ControllerAxis.LEFT_Y);
        telemetry.addData("Right X", axisRightX);
        telemetry.addData("Right Y", axisRightY);
        telemetry.addData("Left X", axisLeftX);
        telemetry.addData("Left Y", axisLeftY);

        // If in tank drive, then set the strafing directions
        if (driveState == DriveTrainState.TANK_TELEOP) {
            if (DrivingGP.getControl(GamePadEx.ControllerButton.LTRIGGER)) { // Strafe left
                direction = Direction.WEST;
            } else if (DrivingGP.getControl(GamePadEx.ControllerButton.RTRIGGER)) { // Strafe right
                direction = Direction.EAST;
            } else { // Not strafing
                direction = Direction.TANK_TELEOP_DRIVE;
            }
        }

        // Perform actions based on the current state
        switch (driveState) {
            case TANK_TELEOP:
                MoveMotorsWithDirection(direction);

            case FIELD_CENTRIC_TELEOP:
                double orientation = Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                double sin = Math.sin(orientation);
                double cos = Math.cos(orientation);
                // forward = y
                // strafe = x
                // clockwise rotation = rx

                double temp = axisLeftY * cos - axisLeftX * sin;
                axisLeftX = axisLeftY * sin + axisLeftX * cos;
                axisLeftY = temp;

                double denominator = Math.max(Math.abs(axisLeftY) + Math.abs(axisLeftX) + Math.abs(axisRightX), 1);
                // this denominator scales the values outside of range [1,-1]
                BR.setPower(0.75 * ((axisLeftY + axisLeftX - axisRightX) / denominator));
                FR.setPower(0.75 * ((axisLeftY - axisLeftX - axisRightX) / denominator));
                BL.setPower(0.75 * ((axisLeftY - axisLeftX + axisRightX) / denominator));
                FL.setPower(0.75 * ((axisLeftY + axisLeftX + axisRightX) / denominator));

                telemetry.addData("heading: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
        }
    }

    public void MoveMotorsWithDirection(Direction dir){
        switch (dir){
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
        MOVE, // Auton, escapable
        TURN, // Auton, escapable
        STOPPING, // Auton, escapable
        IDLE, // Reset encoders. Auton, escapable
        TANK_TELEOP, // TeleOP, NOT escapable
        FIELD_CENTRIC_TELEOP, // TeleOP, NOT escapable
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

    public void setMotorMode(DcMotor.RunMode mode) {
        BR.setMode(mode);
        FR.setMode(mode);
        BL.setMode(mode);
        FL.setMode(mode);
    }

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
}