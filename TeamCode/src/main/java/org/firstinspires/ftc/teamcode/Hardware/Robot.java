package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Robot {
    // Declare driving motors
    private DcMotor FR, FL, BR, BL;

    // Declare system motors (not driving motors)
    private DcMotor intakeRight, intakeLeft, duckSpinner, linSlide;

    public Servo cargoFlipper;
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

        // Assign all system motors (not drive motors) names in the hub
        intakeRight = map.get(DcMotor.class, "intakeRight");
        intakeLeft = map.get(DcMotor.class, "intakeLeft");
        duckSpinner = map.get(DcMotor.class, "duckSpinner");
        linSlide = map.get(DcMotor.class, "linSlide");

        cargoFlipper = map.get(Servo.class, "cargoFlipper");

        // Set motor direction according to their orientation on the bot
        //   motors on the left side will be reversed so that their directions coorespond to
        //      the motors on the right
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set the runmode of the driving motors to use encoders
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the direction of the system motors based on their orientation on the bot
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        duckSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all system motors to run using encoders
//        intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // If this method is being called from an auton function set the zero power behavior of
        //    all the motors to brake, making lock when there is no power being applied
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

    // Turning by making right side motors go backwards and left side motors go forward
    //    positive power will go to the left and negative will go to the left
    public void turn(double power) {
        BR.setPower(-power);
        FR.setPower(-power);
        BL.setPower(power);
        FL.setPower(power);
    }

    // Sets the motor power of all the drive motors to 0
    public void stop() {
        BR.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        FL.setPower(0.0);
    }

    // Sets the mode for the motors (this is how it should be using the encoders)
    public void setMode(DcMotor.RunMode mode){
        BR.setMode(mode);
        FR.setMode(mode);
        BL.setMode(mode);
        FL.setMode(mode);
    }

    // Runs the intake motors at a given power, positive will suck cargo and negative will
    //     spit it out
    public void runIntake(double power){
        intakeRight.setPower(power);
        intakeLeft.setPower(power);
    }

    public void runDuckSpinner(double power){ duckSpinner.setPower(power); }

    public void runLinSlide(double power){ linSlide.setPower(power); }

    // Adjusts the heading of the bot using gyroscope, degree amount to turn and motor power
    public boolean adjustHeading(int degrees, double power, BNO055IMU imu) {
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
        if (Math.abs(difference) <= .5) {
            telemetry.addData("Stopping the ", "bot");
            stop();
            return true;
        }

        // return false otherwise
        return false;
    }

    // sets the target position of the robot given the direction and target position
    public int autonDrive(MovementEnum movement, int target) {
        // will be current position of the highest encoder value
        int curPos = -1;
        switch (movement) {
            case FORWARD:
                FL.setTargetPosition(target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(target);
                curPos = Math.max(-BR.getCurrentPosition(), Math.max(-BL.getCurrentPosition(), Math.max(FR.getCurrentPosition(), FL.getCurrentPosition())));
                break;

            case BACKWARD:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(-target);
                curPos = Math.max(BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(-FR.getCurrentPosition(), -FL.getCurrentPosition())));
                break;

            case LEFTSTRAFE:
                FL.setTargetPosition(target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(-target);
                curPos = Math.max(BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(FR.getCurrentPosition(), FL.getCurrentPosition())));
                break;

            case RIGHTSTRAFE:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(target);
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
}