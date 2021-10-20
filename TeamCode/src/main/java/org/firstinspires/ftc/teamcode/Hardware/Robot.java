package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private DcMotor FR, FL, BR, BL;
    private HardwareMap map;
    private Telemetry telemetry;
    private boolean isAuton;

    public boolean stopped;
    public boolean isStopping;
    public boolean atTargetDriveSpeed;
    public boolean isEasingDriveSpeed;
    public boolean atTargetTurnSpeed;
    public boolean isEasingTurnSpeed;

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

    public void driveEased(double rightStickY, double leftStickY, int easeStage) {
        setPowerEased(BR, rightStickY, easeStage, false);
        setPowerEased(FR, rightStickY, easeStage, false);
        setPowerEased(BL, leftStickY, easeStage, false);
        setPowerEased(FL, leftStickY, easeStage, false);
    }

    public void turn(double power) {
        BR.setPower(-power);
        FR.setPower(-power);
        BL.setPower(power);
        FL.setPower(power);
    }

    public void turnEased(double power, double easeStage) {
        setPowerEased(BR, -power, easeStage, true);
        setPowerEased(FR, -power, easeStage, true);
        setPowerEased(BL, power, easeStage, true);
        setPowerEased(FL, power, easeStage, true);
    }

    public void stop() {
        BR.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        FL.setPower(0.0);
    }

    public void stopEased(double easeStage) {
        BR.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        FL.setPower(0.0);

        setPowerEased(BR, 0.0, easeStage, false);
        setPowerEased(FR, 0.0, easeStage, false);
        setPowerEased(BL, 0.0, easeStage, false);
        setPowerEased(FL, 0.0, easeStage, false);
    }

    // Ease in motor power by interpolating between the current power and the goal power using a stage variable
    public void setPowerEased(Object motorObject, double goalPower, double easeStage, boolean isTurn) {
        DcMotor motor = (DcMotor) motorObject;

        if (motor != null) {
            double currPower = motor.getPower();
            double lerpedPower;
            lerpedPower = lerpValue(currPower, goalPower, easeStage);

            telemetry.addData(("goalPower for: " + motor.toString()), goalPower);
            telemetry.addData(("currPower for: " + motor.toString()), currPower);
            telemetry.addData(("easeStage for: " + motor.toString()), easeStage);
            telemetry.addData(("lerpedPower for: " + motor.toString()), lerpedPower);
            motor.setPower(lerpedPower);
            if (lerpedPower == goalPower) {
                stopped = false;
                if (isTurn) {
                    if (goalPower == 0) {
                        isStopping = false;
                        stopped = true;
                    } else {
                        isEasingTurnSpeed = false;
                        atTargetTurnSpeed = true;
                    }
                } else {
                    if (goalPower == 0) {
                        isEasingDriveSpeed = false;
                        stopped = true;
                    } else {
                        isEasingDriveSpeed = false;
                        atTargetDriveSpeed = true;
                    }
                }
            } else {
                stopped = false;
                if (isTurn) {
                    isEasingTurnSpeed = true;
                    atTargetTurnSpeed = false;
                } else {
                    if (goalPower == 0) {
                        isStopping = true;
                    } else {
                        isEasingDriveSpeed = true;
                        atTargetDriveSpeed = false;
                    }
                }
            }
        }
    }

    double lerpValue(double a, double b, double t) {
        return (a * (1.0 - t)) + (b * t);
    }
}