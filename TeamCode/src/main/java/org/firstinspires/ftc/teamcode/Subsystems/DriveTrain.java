package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    // auton variables
    private ElapsedTime runtime;
    private int target;
    private int angle;

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
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initImu();
        runtime = new ElapsedTime();

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
        switch(driveState){
            case MOVE:
                drivePower = 0.5;
                moveMotorsWithDirection(direction);
                if(Math.abs(BR.getCurrentPosition()) > target){
                    runtime.reset();
                    driveState = DriveTrainState.STOPPING;
                }
                break;

            case TURN:
                if (FL.getMode().equals(DcMotor.RunMode.STOP_AND_RESET_ENCODER)) {
                    FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if(adjustHeading(angle, 0.5) & runtime.milliseconds()>1500){
                    driveState = DriveTrainState.STOPPING;
                }
                break;

            case STOPPING:
                stop();
                setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if(runtime.milliseconds() > 500){
                    driveState = DriveTrainState.IDLE;
                }
                break;

            case IDLE:
                runtime.reset();
                break;
        }
    }

    @Override
    public void updateTeleOpState(GamePadEx GP1, GamePadEx GP2){
        if (FL.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        axisRightY = GP1.getAxis(GamePadEx.ControllerAxis.RIGHT_Y);
        axisLeftY = GP2.getAxis(GamePadEx.ControllerAxis.LEFT_Y);

        BR.setPower(axisRightY);
        FR.setPower(axisRightY);
        BL.setPower(axisLeftY);
        FL.setPower(axisLeftY);

        if(GP1.getControl(GamePadEx.ControllerButton.LTRIGGER)){
            direction = Direction.WEST;
        } else if(GP1.getControl(GamePadEx.ControllerButton.RTRIGGER)){
            direction = Direction.EAST;
        }
    }

    public void updateTeleOpStateOLD(GamePadEx DrivingGP, GamePadEx OtherGP) {
//        if (driveState == DriveTrainState.IDLE) { // CHANGE THIS, ONLY FOR TESTING
//            driveState = DriveTrainState.TANK_TELEOP;
//            direction = Direction.TANK_TELEOP_DRIVE;
//        }

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
                drivePower = DrivingGP.getAxis(GamePadEx.ControllerAxis.LEFT_TRIGGER);
                direction = Direction.WEST;
            } else if (DrivingGP.getControl(GamePadEx.ControllerButton.RTRIGGER)) { // Strafe right
                drivePower = DrivingGP.getAxis(GamePadEx.ControllerAxis.RIGHT_TRIGGER);
                direction = Direction.EAST;
            } else { // Not strafing
                direction = Direction.TANK_TELEOP_DRIVE;
            }
        }

        // Perform actions based on the current state
        switch (driveState) {
            case TANK_TELEOP:
                moveMotorsWithDirection(direction);

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

    public void moveMotorsWithDirection(Direction dir){
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

        // If the bot is within a 30 degree threshold of the target, slow it down to 25% of the desired speed to prevent overshooting
        if (Math.abs(difference) <= 30) {
            FR.setPower(power / 10);
            BR.setPower(power / 10);
            FL.setPower(power / 10);
            BL.setPower(power / 10);
        } else { // Otherwise use normal speed
            FR.setPower(power / 10);
            BR.setPower(power / 10);
            FL.setPower(power / 10);
            BL.setPower(power / 10);
        }

        // If the bot is within 1 degree of the target, stop the bot and return true
        if (Math.abs(difference) <= 0.5) {
            telemetry.addLine("Stopping the bot");
            stop();
            return true;
        }

        // return false otherwise
        return false;
    }
}