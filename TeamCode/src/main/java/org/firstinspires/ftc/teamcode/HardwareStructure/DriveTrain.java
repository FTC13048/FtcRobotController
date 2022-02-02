package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class DriveTrain extends Subsystems {
    private DcMotor FL, FR, BL, BR;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;

    //region Movment Stats
    private DriveTrainState driveState;
    private Direction direction;
    private double drivePower;
    //endregion

    //region Dependent Classes
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    //endregion

    public DriveTrain(HardwareMap hmap, Telemetry tele, boolean isAuton) {
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

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (isAuton) {
            initImu();

            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("drive train", "initialized");
    }

    @Override
    public void updateState() {

    }

    @Override
    public void updateTeleopState(GamePadEx gp1, GamePadEx gp2) {
        DoTeleOp(gp1, gp2); // Justification: Yes
    }

    public void DoTeleOp(GamePadEx gp1, GamePadEx gp2) {
        if (driveState == DriveTrainState.IDLE) {
            driveState = DriveTrainState.TANK_TELEOP;
        }


        // stuff
        if (driveState == DriveTrainState.TANK_TELEOP) {
            if(gp1.)

            MoveMotorsWithDir(direction);

        } else if (driveState == DriveTrainState.FIELD_CENTRIC_TELEOP) {

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
        NONE, // Fallback or no movement
    }

    public void MoveMotorsWithDir(Direction dir) {
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
        }
    }

    public void setMode(DcMotor.RunMode mode) {
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

    private void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }
}
