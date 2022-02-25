package org.firstinspires.ftc.teamcode.OldHardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {
    // Declare driving motors
    private DcMotor FR, FL, BR, BL;

    // Declare system motors (not driving motors)
    public DcMotor intakeRight, intakeLeft, duckSpinner, linSlide;
    private ElapsedTime timer;

    public Servo cargoFlipper;
    private ModernRoboticsI2cRangeSensor distSensorBack;
    private ModernRoboticsI2cRangeSensor distSensorLeft;
    private ModernRoboticsI2cRangeSensor distSensorRight;

    private HardwareMap map;
    private Telemetry telemetry;
    private boolean isAuton;

    public final int FIRST_LEVEL = -892;
    public final int SECOND_LEVEL = -1063;
    public final int THIRD_LEVEL = -1550;

    public Robot(HardwareMap hmap, Telemetry tele, boolean auton) {
        this.map = hmap;
        telemetry = tele;
        isAuton = auton;
    }

    // initializes all key systems of the bot (motors, servos, etc.)
    public void initBot() {
        timer = new ElapsedTime();
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
        intakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // If this method is being called from an auton function set the zero power behavior of
        //    all the motors to brake, making lock when there is no power being applied
        if (isAuton) {
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            distSensorBack = map.get(ModernRoboticsI2cRangeSensor.class, "distSensorBack");
            distSensorLeft = map.get(ModernRoboticsI2cRangeSensor.class, "distSensorLeft");
            distSensorRight = map.get(ModernRoboticsI2cRangeSensor.class, "distSensorRight");
        }
    }

    public void start() {
        intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void drive(double rightSide, double leftSide) {
        BR.setPower(rightSide);
        FR.setPower(rightSide);
        BL.setPower(leftSide);
        FL.setPower(leftSide);
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
    public void setMode(DcMotor.RunMode mode) {
        BR.setMode(mode);
        FR.setMode(mode);
        BL.setMode(mode);
        FL.setMode(mode);
    }

    // Runs the intake motors at a given power, positive will suck cargo and negative will
    //     spit it out
    public void runIntake(double power) {
        intakeRight.setPower(power);
        intakeLeft.setPower(power);
    }

    public boolean setLinSlidePos(int numTicks) {
        linSlide.setTargetPosition(numTicks);
        linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runLinSlide(0.4);

        if (Math.abs(linSlide.getCurrentPosition() - numTicks) <= 5) {
            runLinSlide(0.0);
            linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return true;
        }

        return false;
    }

    public void runDuckSpinner(double power) {
        duckSpinner.setPower(power);
    }

    public void runLinSlide(double power) {
        linSlide.setPower(power);
    }

    public int getLinSlidePos() {
        return linSlide.getCurrentPosition();
    }

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

        // If the bot is within a 30 degree threshold of the target, slow it down to 25% of the desired speed to prevent overshooting
        if (Math.abs(difference) <= 30) {
            turn(power / 10);
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

    // sets the target position of the robot given the direction and target position
    public int autonDrive(MovementEnum movement, int target) {
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

    private boolean firstPass = true;
    public boolean delay(double seconds){
        if(firstPass){
            timer.reset();
            firstPass = false;
        }

        if(!firstPass){
            if(timer.seconds() >= seconds){ return true; }
        }

        return false;
    }

    public void wheelTelem() {
        telemetry.addData("FR", FR.getCurrentPosition());
        telemetry.addData("FL", FL.getCurrentPosition());
        telemetry.addData("BR", BR.getCurrentPosition());
        telemetry.addData("BL", BL.getCurrentPosition());
    }

    // ---------------------------- DIST SENSOR DRIVING ----------------------------

    public double getBackDistanceCM() {
        return Range.clip(distSensorBack.getDistance(DistanceUnit.CM), 0.0, 200.0);
    }

    public double getRightDistanceCM() {
        return Range.clip(distSensorRight.getDistance(DistanceUnit.CM), 0.0, 200.0);
    }

    public double getLeftDistanceCM() {
        return Range.clip(distSensorLeft.getDistance(DistanceUnit.CM), 0.0, 200.0);
    }

    // drives until distance sensor reads a certain distance and then returns true when there
    public boolean driveBackDistanceSensor(double stopDistCM, double power, MovementEnum movement) {
        // if the sensor reads the stop distance return true
        //    if it reads 10 cm within the stop distance, set the motor power to 6 times
        //    less than the entered power
        double distance = getBackDistanceCM();
        if (Math.abs(distance - stopDistCM) < 3) {
            stop();
            return true;
        } else {
            if (Math.abs(distance - stopDistCM) <= 15) {
                power = 0.1;
            }

            power *= -1 * Math.signum(distance - stopDistCM);
            switch (movement) {
                case FORWARD:
                    drive(-power, -power);
                    break;

                case BACKWARD:
                    drive(power, power);
                    break;

                case LEFTSTRAFE:
                    strafe(power);
                    break;

                case RIGHTSTRAFE:
                    strafe(-power);
                    break;
            }

            return false;
        }
    }

    // drives until distance sensor reads a certain distance and then returns true when there
    public boolean driveRightDistanceSensor(double stopDistCM, double power, MovementEnum movement) {
        // if the sensor reads the stop distance return true
        //    if it reads 5 inches within the stop distance, set the motor power to 6 times
        //    less than the entered power
        double distance = getRightDistanceCM();
        if (Math.abs(distance - stopDistCM) < 1) {
            stop();
            return true;
        } else {
            if (Math.abs(distance - stopDistCM) <= 15) {
                power = 0.1;
            }

            power *= -1 * Math.signum(distance - stopDistCM);
            switch (movement) {
                case FORWARD:
                    drive(-power, -power);
                    break;

                case BACKWARD:
                    drive(power, power);
                    break;

                case LEFTSTRAFE:
                    strafe(power);
                    break;

                case RIGHTSTRAFE:
                    strafe(-power);
                    break;
            }

            return false;
        }
    }

    // drives until distance sensor reads a certain distance and then returns true when there
    public boolean driveLeftDistanceSensor(double stopDistCM, double power, MovementEnum movement) {
        // if the sensor reads the stop distance return true
        //    if it reads 5 inches within the stop distance, set the motor power to 6 times
        //    less than the entered power
        double distance = getLeftDistanceCM();
        if (Math.abs(distance - stopDistCM) <= 1) {
            stop();
            return true;
        } else {
            if (Math.abs(distance - stopDistCM) <= 15) {
                power = 0.1;
            }

            power *= -1 * Math.signum(distance - stopDistCM);
            switch (movement) {
                case FORWARD:
                    drive(-power, -power);
                    break;

                case BACKWARD:
                    drive(power, power);
                    break;

                case LEFTSTRAFE:
                    strafe(power);
                    break;

                case RIGHTSTRAFE:
                    strafe(-power);
                    break;
            }

            return false;
        }
    }
}