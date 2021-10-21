package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Robot {
    private DcMotor FR, FL, BR, BL;
    private HardwareMap map;
    private Telemetry telemetry;
    private boolean isAuton;

    public Robot(HardwareMap hmap, Telemetry tele, boolean auton) {
        this.map = hmap;
        telemetry = tele;
        isAuton = auton;
    }

    // initializes all key systems of the bot (motors, servos, etc.)
    public void initBot() {
        // Assign all motors to ports, name it this exact thing in the driver hub
        BR = map.get(DcMotor.class, "back_right");
        FR = map.get(DcMotor.class, "front_right");
        BL = map.get(DcMotor.class, "back_left");
        FL = map.get(DcMotor.class, "front_left");

        // Set motor direction according to their orientation on the bot
        //   motors on the left side will be reversed so that their directions coorespond to
        //      the motors on the right
        // REMEMBER TO COMMENT OUT SECOND AND UNCOMMENT FIRST FOR CHAIN MOTORS
//        BR.setDirection(DcMotorSimple.Direction.FORWARD);
//        BL.setDirection(DcMotorSimple.Direction.REVERSE);
//        FL.setDirection(DcMotorSimple.Direction.REVERSE);
//        FR.setDirection(DcMotorSimple.Direction.FORWARD);

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (isAuton) {
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // Sets the strafing power for the robot (negative number will strafe left and positive
    //     will strafe right)
    public void strafe(double strafePower) {
        BR.setPower(-strafePower);
        FR.setPower(strafePower);
        BL.setPower(strafePower);
        FL.setPower(-strafePower);
    }

    // Sets all motors to drive with the given power (positive forward negative backward)
    public void drive(double rightStickY, double leftStickY) {
        BR.setPower(rightStickY);
        FR.setPower(rightStickY);
        BL.setPower(leftStickY);
        FL.setPower(leftStickY);
    }

    public void turn(double power) {
        BR.setPower(-power);
        FR.setPower(-power);
        BL.setPower(power);
        FL.setPower(power);
    }

    public void stop() {
        BR.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        FL.setPower(0.0);
    }

    public boolean adjustHeading(int degrees, double power, BNO055IMU imu) {
        // get the current heading of the bot (an angle from -180 to 180)
        int currHeading = (int) imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

        // to convert to a 0-360 scale, if the current heading is negative add
        //    360 to it
        currHeading = currHeading < 0 ? 360 + currHeading : currHeading;

        // difference between target and current heading
        double difference = degrees - currHeading;
        telemetry.addData("Difference is ", difference);

        // If the bot is within 30 degrees of the target, slow it down to 25% of the desired speed to prevent overshooting
        if (difference <= 30) {
            turn(power / 4);
        } else { // Otherwise use normal speed
            turn(power);
        }


        // If the bot is within 1 degree of the target, stop the bot and return true
        if (difference <= 1) {
            telemetry.addData("Stopping the ", "bot");
            stop();
            return true;
        }

        // return false otherwise
        return false;
    }
}