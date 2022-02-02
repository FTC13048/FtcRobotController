package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;

public class DriveTrain extends Subsystems {

    //region Physical Components
    private DcMotor FL, FR, BL, BR;
    private BNO055IMU directionSensor;
    private BNO055IMU.Parameters directionSensorParameters;
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

        initializeDirectionSensor();

        if (isAuton) { // Set the motors to brake for ONLY auton
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

    public void InitLoopAuton() {
        // gets data for robot orientation
        cbgyro.initLoopGyro(directionSensor, telemetry);
    }

    @Override
    public void updateTeleopState(GamePadEx DrivingGP, GamePadEx OtherGP) {
        DoTeleOp(DrivingGP, OtherGP); // Justification: Yes
    }

    public void DoTeleOp(GamePadEx DrivingGP, GamePadEx OtherGP) {
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

            MoveMotorsWithDir(direction, DrivingGP);
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

            double orientation = Math.toRadians(directionSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
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

            telemetry.addData("heading: ", directionSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            //telemetry.addData("FL", leftFront.getCurrentPosition());
            //telemetry.addData("BL", leftBack.getCurrentPosition());
            //telemetry.addData("FR", leftFront.getCurrentPosition());
            //telemetry.addData("BR", rightBack.getCurrentPosition());
            telemetry.update();

            MoveMotorsWithDir(direction, DrivingGP);
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

    public void MoveMotorsWithDir(Direction dir, GamePadEx gp) {
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

    // Declare OpMode members.
    private CalibrateGyro cbgyro;

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

    private void initializeDirectionSensor() {
//        directionSensor = hardwareMap.get(BNO055IMU.class, "imu");
//        directionSensorParameters = new BNO055IMU.Parameters();

        cbgyro = new CalibrateGyro(true);
        directionSensor = cbgyro.initGyro(hardwareMap);
//        directionSensor.initialize(directionSensorParameters);
    }
}

class CalibrateGyro {

    final static String FILENAME = "BNO055IMUCalibration.json";

    private boolean readFromFile;
    private boolean writeComplete;

    /**
     * Construct a new CalibrateGyro object
     *
     * @param readFromFile true = read calibration data (teleop),
     *                     false = generate calibration data (auton)
     */
    public CalibrateGyro(boolean readFromFile) {
        this.readFromFile = readFromFile;
        this.writeComplete = false;
    }

    /**
     * Call this in init() as @code BNO055IMU imu = CalibrateGyro.initGyro(....);
     * NOTE: Above is why there was null pointer problem in tuesday's code --- MAKE SURE TO SET EQUAL ^^^
     *
     * @param hwMap      OpMode HardwareMap instance
     * @param configName OPTIONAL: name for configuration, by default "imu"
     * @return An instance of your gyro, fully initialized (but not necessarily done calibrating!)
     */
    public BNO055IMU initGyro(HardwareMap hwMap, String configName) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Enable logging
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        // If read calibration data from file, set param to filename
        if (this.readFromFile) {
            //File file = AppUtil.getInstance().getSettingsFile(FILENAME);
            //parameters.calibrationData = BNO055IMU.CalibrationData.deserialize(ReadWriteFile.readFile(file));
            parameters.calibrationDataFile = FILENAME;
        }

        // Initialize gyro with either data from file or new data
        BNO055IMU gyro = hwMap.get(BNO055IMU.class, configName);

        if (!this.readFromFile) {
            gyro.initialize(parameters);
        }

        return gyro;
    }

    // Helper so u can do less work -- uses default name for imu in config
    public BNO055IMU initGyro(HardwareMap hwMap) {
        return initGyro(hwMap, "imu");
    }

    /**
     * CALL THIS IN initLoop() once per loop
     *
     * @param gyro      your gyro instance
     * @param telemetry OpMode telemetry instance
     */
    public void initLoopGyro(BNO055IMU gyro, Telemetry telemetry) {

        // If we don't want to read from file, the gyro is calibrated,
        // and we have not already written to a file...
        if (!this.readFromFile && gyro.isGyroCalibrated() && !this.writeComplete) {
            // get calibration data from imu
            BNO055IMU.CalibrationData calibrationData = gyro.readCalibrationData();

            // write it to a json file
            File file = AppUtil.getInstance().getSettingsFile(FILENAME);
            ReadWriteFile.writeFile(file, calibrationData.serialize());

            // state that writing is complete
            this.writeComplete = true;
        }

        // output things about the gyro in telemetry
        for (String s : getCalibrationInfo(gyro)) {
            telemetry.addLine(s);
        }

        // output if the file write is complete -- ONLY PRESS START ON AUTON ONCE THIS SHOWS UP
        if (this.writeComplete) {
            telemetry.addLine("saved to " + FILENAME);
        }

        // update telemetry
        //telemetry.update();
    }

    private static List<String> getCalibrationInfo(BNO055IMU gyro) {
        List<String> telemetryData = new ArrayList<>();
        telemetryData.add("Sensor: IMU Gyro -----------");
        telemetryData.add("Status: " + gyro.getSystemStatus().toShortString());
        telemetryData.add("Calib Status: " + gyro.getCalibrationStatus().toString());
        telemetryData.add("Gyro Calib? " + gyro.isGyroCalibrated());
        telemetryData.add("heading: " + gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

        return telemetryData;
    }
}