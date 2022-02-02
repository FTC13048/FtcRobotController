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
        TeleOPStuff(gp1, gp2); // Justification: Yes
    }

    public void TeleOPStuff(GamePadEx gp1, GamePadEx gp2) {
        if (driveState == DriveTrainState.IDLE) {
            driveState = DriveTrainState.TANK_TELEOP;
        }

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
        MOVE,
        TURN,
        STOPPING,
        IDLE, // Reset encoders
        TANK_TELEOP,
        FIELD_CENTRIC_TELEOP,
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
        NONE, // Fallback
    }

    public void MoveMotorsWithDir(Direction dir) {
        if (dir == Direction.NORTH) { //
            BR.setPower(drivePower);
            FR.setPower(drivePower);
            BL.setPower(-drivePower);
            FL.setPower(-drivePower);
        } else if (dir == Direction.NORTHEAST) { //
            BR.setPower(drivePower);
            FR.setPower(-drivePower);
            BL.setPower(drivePower);
            FL.setPower(-drivePower);
        } else if (dir == Direction.EAST) { //
            BR.setPower(-drivePower);
            FR.setPower(drivePower);
            BL.setPower(drivePower);
            FL.setPower(-drivePower);
        } else if (dir == Direction.SOUTHEAST) { //
            BR.setPower(-drivePower);
            FR.setPower(-drivePower);
            BL.setPower(-drivePower);
            FL.setPower(-drivePower);
        } else if (dir == Direction.SOUTH) { //
            BR.setPower(-drivePower);
            FR.setPower(-drivePower);
            BL.setPower(drivePower);
            FL.setPower(drivePower);
        } else if (dir == Direction.SOUTHWEST) { //
            BR.setPower(-drivePower);
            FR.setPower(drivePower);
            BL.setPower(-drivePower);
            FL.setPower(drivePower);
        } else if (dir == Direction.WEST) { //
            BR.setPower(drivePower);
            FR.setPower(-drivePower);
            BL.setPower(-drivePower);
            FL.setPower(drivePower);
        } else if (dir == Direction.NORTHWEST) { //
            BR.setPower(drivePower);
            FR.setPower(drivePower);
            BL.setPower(drivePower);
            FL.setPower(drivePower);
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

    // drives until distance sensor reads a certain distance and then returns true when there
    public boolean driveDistanceSensor(double stopDist, double power,
                                       DistanceSensor sensor, DirectionEnum movement) {
        // if the sensor reads the stop distance return true
        //    if it reads 5 inches within the stop distance, set the motor power to 6 times
        //    less than the entered power
        if (Math.abs(sensor.getDistCM() - stopDist) < 3) {
            stop();
            return true;
        } else {
            if (Math.abs(sensor.getDistCM() - stopDist) <= 15) {
                power /= 6;
            }

            switch (movement) {
                case FORWARD:
                    drive(power, power);
                    break;

                case BACKWARD:
                    drive(-power, -power);
                    break;

                case LEFTSTRAFE:
                    strafe(-power);
                    break;

                case RIGHTSTRAFE:
                    strafe(power);
                    break;
            }

            return false;
        }
    }

    // sets the target position of the robot given the direction and target position
    public int autonDrive(DirectionEnum movement, int target) {
        // will be current position of the highest encoder value
        int curPos = -1;
        switch (movement) {
            case FORWARD:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(-target);
                curPos = Math.max(-BR.getCurrentPosition(), Math.max(-BL.getCurrentPosition(), Math.max(FR.getCurrentPosition(), FL.getCurrentPosition())));
                break;

            case BACKWARD:
                FL.setTargetPosition(target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(target);
                curPos = Math.max(BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(-FR.getCurrentPosition(), -FL.getCurrentPosition())));
                break;

            case LEFTSTRAFE:
                FL.setTargetPosition(target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(target);
                curPos = Math.max(BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(FR.getCurrentPosition(), FL.getCurrentPosition())));
                break;

            case RIGHTSTRAFE:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(-target);
                curPos = Math.max(-BR.getCurrentPosition(), Math.max(-BL.getCurrentPosition(), Math.max(-FR.getCurrentPosition(), -FL.getCurrentPosition())));
                break;

            case STOP:
                FL.setTargetPosition(FL.getCurrentPosition());
                FR.setTargetPosition(FR.getCurrentPosition());
                BL.setTargetPosition(BL.getCurrentPosition());
                BR.setTargetPosition(BR.getCurrentPosition());
                break;
        }

        return curPos;
    }

    // Adjusts the heading of the bot using gyroscope, degree amount to turn and motor power
    public boolean adjustHeading(int degrees, double power) {
        // get the current heading of the bot (an angle from -180 to 180)
        float currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // to convert to a 0-360 scale, if the current heading is negative add
        //    360 to it
        currHeading = currHeading < 0 ? 360 + currHeading : currHeading;

        // difference between target and current heading
        double difference = degrees - currHeading;
        telemetry.addData("Difference is ", difference);

        // If the bot is within 30 degrees of the target, slow it down to 25% of the desired speed to prevent overshooting
        if (Math.abs(difference) <= 30) {
            turn(power / 6);
        } else { // Otherwise use normal speed
            turn(power);
        }

        // If the bot is within 1 degree of the target, stop the bot and return true
        if (Math.abs(difference) <= 0.5) {
            telemetry.addData("Stopping the ", "bot");
            stop();
            return true;
        }

        // return false otherwise
        return false;
    }

    private void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }
}
