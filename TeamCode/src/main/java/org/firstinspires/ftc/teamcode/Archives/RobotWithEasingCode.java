package org.firstinspires.ftc.teamcode.Archives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotWithEasingCode {
    private DcMotor FR, FL, BR, BL;
    private HardwareMap map;
    private Telemetry telemetry;
    private boolean isAuton;

    // Easing stuff
//    public boolean stopped;
//    public boolean isStopping;
//    public boolean atTargetDriveSpeed;
//    public boolean isEasingDriveSpeed;
//    public boolean atTargetTurnSpeed;
//    public boolean isEasingTurnSpeed;
//    public double startPowerFR, startPowerFL, startPowerBR, startPowerBL;

    public RobotWithEasingCode(HardwareMap hmap, Telemetry tele, boolean auton) {
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

//    public void driveEased(double rightStickY, double leftStickY, int easeProgress) {
//        BR.setPower(setPowerEased(startPowerBR, rightStickY, easeProgress, false));
//        FR.setPower(setPowerEased(startPowerFR, rightStickY, easeProgress, false));
//        BL.setPower(setPowerEased(startPowerBL, leftStickY, easeProgress, false));
//        FL.setPower(setPowerEased(startPowerFL, leftStickY, easeProgress, false));
//    }

    public void turn(double power) {
        BR.setPower(-power);
        FR.setPower(-power);
        BL.setPower(power);
        FL.setPower(power);
    }

//    public void turnEased(double goalPower, double easeProgress) {
//        BR.setPower(setPowerEased(startPowerBR, -goalPower, easeProgress, true));
//        FR.setPower(setPowerEased(startPowerFR, -goalPower, easeProgress, true));
//        BL.setPower(setPowerEased(startPowerBL, goalPower, easeProgress, true));
//        FL.setPower(setPowerEased(startPowerFL, goalPower, easeProgress, true));
//    }

    public void stop() {
        BR.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        FL.setPower(0.0);
    }

//    public void stopEased(double easeProgress) {
//        BR.setPower(setPowerEased(startPowerBR, 0.0, easeProgress, false));
//        FR.setPower(setPowerEased(startPowerFR, 0.0, easeProgress, false));
//        BL.setPower(setPowerEased(startPowerBL, 0.0, easeProgress, false));
//        FL.setPower(setPowerEased(startPowerFL, 0.0, easeProgress, false));
//    }

    // Set the start power variables
//    public void setStartPowers() {
//        startPowerFR = FR.getPower();
//        startPowerFL = FL.getPower();
//        startPowerBR = BR.getPower();
//        startPowerBL = BL.getPower();
//    }

    // Ease in motor power by interpolating between the current power and the goal power using a stage variable
    // startPower is the start power that we want to ease from
    // goalPower is the target power that we want to reach after easing
    // easeProgress is the progress so far in the ease process, this is passed from the loop function in auton.java because it has to go up every loop call
    // isTurn is true if you want to use this function to turn and its false if you want to use it to drive
//    public double setPowerEased(double startPower, double goalPower, double easeProgress, boolean isTurn) {
//        double lerpedPower;
//
//        // Use the lerp function to get the new power
//        lerpedPower = lerpValue(startPower, goalPower, easeProgress);
//        telemetry.addData("lerpedPower UNCLAMPED", lerpedPower);
//
//        // Clamp the lerpedValue between the start and goals powers
//        if (goalPower < 0)
//            lerpedPower = -clamp(lerpedPower, startPower, goalPower);
//        else
//            lerpedPower = clamp(lerpedPower, startPower, goalPower);
//
//        // Print a bunch of useful stuff for debugging
//        telemetry.addData("easeProgress CLAMPED", easeProgress);
//        telemetry.addData("lerpedPower", lerpedPower);
////        telemetry.addData("isEasingTurnSpeed", isEasingTurnSpeed);
////        telemetry.addData("atTargetTurnSpeed", atTargetTurnSpeed);
////        telemetry.addData("isEasingDriveSpeed", isEasingDriveSpeed);
////        telemetry.addData("atTargetDriveSpeed", atTargetDriveSpeed);
////        telemetry.addData("isStopping", isStopping);
////        telemetry.addData("stopped", stopped);
//
//        // Set the boolean states to the current state of the ease
//        if (lerpedPower == goalPower) { // Ease process is done
//            telemetry.addData("Done", "easing");
//
//            if (isTurn) {
//                if (goalPower == 0) {
//                    isStopping = false;
//                    stopped = true;
//                } else {
//                    isEasingTurnSpeed = false;
//                    atTargetTurnSpeed = true;
//                }
//            } else {
//                if (goalPower == 0) {
//                    isEasingDriveSpeed = false;
//                    stopped = true;
//                } else {
//                    isEasingDriveSpeed = false;
//                    atTargetDriveSpeed = true;
//                }
//            }
//        } else { // Ease process is NOT done
//            telemetry.addData("NOT done", "easing");
//            stopped = false;
//            if (isTurn) {
//                isEasingTurnSpeed = true;
//                atTargetTurnSpeed = false;
//            } else {
//                if (goalPower == 0) {
//                    isStopping = true;
//                    stopped = false;
//                } else {
//                    isEasingDriveSpeed = true;
//                    atTargetDriveSpeed = false;
//                }
//            }
//        }
//        return lerpedPower;
//    }

    // Make sure a number is inside a defined range
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // Interpolate between two values, a and b, using the time value, t (between 0 to 1)
    public double lerpValue(double a, double b, double t) {
        return (a * (1.0 - t)) + (b * t);
    }
}