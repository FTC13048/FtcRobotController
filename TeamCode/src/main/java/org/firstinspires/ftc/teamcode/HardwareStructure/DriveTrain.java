package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class DriveTrain extends Subsystems {

    //region Physical Components
    private DcMotor FL, FR, BL, BR;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    //endregion

    //region Movement Stats
    private DriveTrainState driveState;
    private Direction direction;
    private double drivePower;
    //endregion

    //region Dependent Classes
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    //endregion

    public DriveTrain(HardwareMap hmap, Telemetry tele, boolean isAuton) { // REMOVE THE ISAUTON BOOLEAN
        super(tele);

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
        }

        telemetry.addData("Drive Train", "initialized");
    }

    @Override
    public void updateState() {

    }

    @Override
    public void updateTeleopState(GamePadEx DrivingGP, GamePadEx OtherGP) {
        if (driveState == DriveTrainState.IDLE) {
            driveState = DriveTrainState.TANK_TELEOP; // CHANGE THIS, ONLY FOR TESTING
        }

        // Stuff
        if (driveState == DriveTrainState.TANK_TELEOP) {
            if (DrivingGP.getControl(GamePadEx.ControllerButton.LTRIGGER)) { // Strafe left
                direction = Direction.WEST;
            } else if (DrivingGP.getControl(GamePadEx.ControllerButton.RTRIGGER)) { // Strafe right
                direction = Direction.EAST;
            } else { // Not strafing
                direction = Direction.TANK_TELEOP_DRIVE;
            }

            moveMotorsWithDir(direction, DrivingGP);
        } else if (driveState == DriveTrainState.FIELD_CENTRIC_TELEOP) { // Actually uses cardinal directions from gyroscope
            double axisRightX = DrivingGP.getAxis(GamePadEx.ControllerAxis.RIGHT_X);
            double axisRightY = DrivingGP.getAxis(GamePadEx.ControllerAxis.RIGHT_Y);
            double axisLeftX = DrivingGP.getAxis(GamePadEx.ControllerAxis.LEFT_X);
            double axisLeftY = DrivingGP.getAxis(GamePadEx.ControllerAxis.LEFT_Y);
            telemetry.addData("Right X", axisRightX);
            telemetry.addData("Right Y", axisRightY);
            telemetry.addData("Left X", axisLeftX);
            telemetry.addData("Left Y", axisLeftY);

            if (axisLeftY > 0.0) { // Stick is pointed up
                if (axisLeftX > 0.0) { // Stick is pointed up and right
                    direction = Direction.NORTHEAST;
                } else if (axisLeftX < 0.0) { // Stick is pointed up and left
                    direction = Direction.NORTHWEST;
                }
            } else if (axisLeftY < 0.0) { // Stick is pointed down (I know this is a redundant if statement)
                if (axisLeftX > 0.0) { // Stick is pointed down and right
                    direction = Direction.SOUTHEAST;
                } else if (axisLeftX < 0.0) { // Stick is pointed down and left
                    direction = Direction.SOUTHWEST;
                }
            } else { // Stick isn't pointed up or down
                if (axisLeftX > 0.0) { // Stick is pointed only right
                    direction = Direction.EAST;
                } else if (axisLeftX < 0.0) { // Stick is pointed only left
                    direction = Direction.WEST;
                }
            }

            // NEW STUFF!!!!!!!

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

            moveMotorsWithDir(direction, DrivingGP);
        }
    }

    public DriveTrainState getState() {
        return driveState;
    }

    public enum DriveTrainState {
        MOVE, // Auton, escapable
        TURN, // Auton, escapable
        STOPPING, // Auton, escapable
        IDLE, // Reset encoders Auton, escapable
        TANK_TELEOP, // TeleOP, NOT escapable
        FIELD_CENTRIC_TELEOP, // TeleOP, NOT escapable
        NONE, // Fallback
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
        NONE, // Fallback or no movement
    }

    public void moveMotorsWithDir(Direction dir, GamePadEx gp) {
        if (dir == Direction.NORTH) {
            BR.setPower(drivePower);
            FR.setPower(drivePower);
            BL.setPower(drivePower);
            FL.setPower(drivePower);
        } else if (dir == Direction.NORTHEAST) {
            BR.setPower(drivePower);
            FR.setPower(0.0);
            BL.setPower(0.0);
            FL.setPower(drivePower);
        } else if (dir == Direction.EAST) {
            BR.setPower(drivePower);
            FR.setPower(-drivePower);
            BL.setPower(-drivePower);
            FL.setPower(drivePower);
        } else if (dir == Direction.SOUTHEAST) {
            BR.setPower(0.0);
            FR.setPower(-drivePower);
            BL.setPower(-drivePower);
            FL.setPower(0.0);
        } else if (dir == Direction.SOUTH) {
            BR.setPower(-drivePower);
            FR.setPower(-drivePower);
            BL.setPower(-drivePower);
            FL.setPower(-drivePower);
        } else if (dir == Direction.SOUTHWEST) {
            BR.setPower(-drivePower);
            FR.setPower(0.0);
            BL.setPower(0.0);
            FL.setPower(-drivePower);
        } else if (dir == Direction.WEST) {
            BR.setPower(-drivePower);
            FR.setPower(drivePower);
            BL.setPower(drivePower);
            FL.setPower(-drivePower);
        } else if (dir == Direction.NORTHWEST) {
            BR.setPower(0.0);
            FR.setPower(drivePower);
            BL.setPower(drivePower);
            FL.setPower(0.0);
        } else if (dir == Direction.TANK_TELEOP_DRIVE) {
            double axisRightY = gp.getAxis(GamePadEx.ControllerAxis.RIGHT_Y);
            double axisLeftY = gp.getAxis(GamePadEx.ControllerAxis.LEFT_Y);
            telemetry.addData("Right Y", axisRightY);
            telemetry.addData("Left Y", axisLeftY);

            BR.setPower(axisLeftY);
            FR.setPower(axisLeftY);
            BL.setPower(axisRightY);
            FL.setPower(axisRightY);
        }
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