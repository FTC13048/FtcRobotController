package org.firstinspires.ftc.teamcode.Archives;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;

public class Bot {
    // All motors required to drive
    public static DcMotorEx BR, FR, BL, FL;

    // Add any additional DC motors here
    // public static DcMotorEx

    // Add all servo motors here
    // public Servo

    // hardware map of robot and telemetry data
    HardwareMap map;
    Telemetry telemetry;

    // sensor gives orientation of the bot
    public static BNO055IMU gyro;
    public static PIDFCoefficients pid;

    public Bot(){}

    // Method that initializes all motors, digital bot map, and other systems of the bot
    public void init(HardwareMap map, Telemetry tele){
        // Get the digital map of the robot and the ability to print to driver station
        this.map = map;
        telemetry = tele;

        // Initialize the four motors required for driving, device names here must be
        //    identical to the ones listed in the driver hub
        BR = (DcMotorEx) this.map.get(DcMotor.class, "back_right");
        FR = (DcMotorEx) this.map.get(DcMotor.class, "front_right");
        BL = (DcMotorEx) this.map.get(DcMotor.class, "back_left");
        FL = (DcMotorEx) this.map.get(DcMotor.class, "front_left");

        // Set motor direction according to their orientation on the bot
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize any other DC motors
        // Set direction for any other DC motors

        // Initialize any servo motors
        

        telemetry.addData(">", "Robot initialized");
        telemetry.update();

        // Sets all motors to be able to run with encoders, meaning they can stop at more
        //     precise locations
        this.runMotorsWithEncoders();

        // Orientation parameters for the bot
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        tele.addData(">", "Orientation parameters calculated");
        tele.update();

        // Calibrate the gyro in the control hub
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        gyro = this.map.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);
        tele.addData(">", "Gyro Calibrating. Do Not Move!");
        tele.update();
    }

    // Changes how the motors interpret the data being passed to them
    public static void changeRunModeAuton(DcMotor.RunMode runMode){
        BL.setMode(runMode);
        BR.setMode(runMode);
        FL.setMode(runMode);
        FR.setMode(runMode);
    }

    private static void runMotorsWithEncoders(){
        // Set all motors to run with encoders, pid coefficients make it so the
        //     bot starts fast and slows down approaching target location
        BR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        FR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        BL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        FL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);

        // Set all other DC motors to run using encoder
    }

    // Sets the drive power of the bot to the number given (a number from -1 to 1)
    //     A negative value will drive backwards and positive forwards
    public void drive(double drivePower){
        FL.setPower(-drivePower);
        FR.setPower(-drivePower);
        BL.setPower(drivePower);
        BR.setPower(drivePower);
    }

    // Sets the turn speed of the bot to the power given (a number from -1 to 1)
    //    A positive value will strafe right and negative will strafe left
    public void strafe(double strafePower){
        FL.setPower(strafePower);
        FR.setPower(-strafePower);
        BL.setPower(-strafePower);
        BR.setPower(strafePower);
    }

    // Used for auton: moves based on given direction and target distance
    public int autonDrive(MovementEnum movement, int target) {
        // Position of the motor with the greatest position
        int maxPosition = -1;
        switch (movement) {
            case FORWARD:
                FL.setTargetPosition(target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(target);
                maxPosition = Math.max(BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(FR.getCurrentPosition(), FL.getCurrentPosition())));
                break;

            case BACKWARD:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(-target);
                maxPosition = Math.max(BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(-FR.getCurrentPosition(), -FL.getCurrentPosition())));
                break;

            case LEFTSTRAFE:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(-target);
                maxPosition = Math.max(BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(FR.getCurrentPosition(), FL.getCurrentPosition())));
                break;

            case RIGHTSTRAFE:
                FL.setTargetPosition(target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(target);
                maxPosition = Math.max(-BR.getCurrentPosition(), Math.max(-BL.getCurrentPosition(), Math.max(-FR.getCurrentPosition(), -FL.getCurrentPosition())));
                break;

            case LEFTTURN:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(target);
                maxPosition = Math.max(BR.getCurrentPosition(), Math.max(-BL.getCurrentPosition(), Math.max(FR.getCurrentPosition(), -FL.getCurrentPosition())));
                break;

            case RIGHTTURN:
                FL.setTargetPosition(target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(-target);
                maxPosition = Math.max(-BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(-FR.getCurrentPosition(), FL.getCurrentPosition())));
                break;

            case STOP:
                FL.setTargetPosition(FL.getCurrentPosition());
                FR.setTargetPosition(FR.getCurrentPosition());
                BL.setTargetPosition(BL.getCurrentPosition());
                BR.setTargetPosition(BR.getCurrentPosition());
                break;
        }
        return maxPosition;
    }

    // Stops the bot
    public void stop(){ this.drive(0.0); }
}