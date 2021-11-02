package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "gyro", group = "Autonomous")
public class Auton extends OpMode {
    // Figure out ticks per revolution and ticks per inch
    private static final double TICKS_PER_REV = 134.4;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (4.0 * Math.PI);

    // The bot, gyro (with parameters), and distance sensor objects
    Robot bot;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    private ModernRoboticsI2cRangeSensor distSensor;

    // These variables 'debounce' the bumpers. They make sure the user can hold down
    //    the bumper for longer than a tick without the system repeatedly switching the value
    private boolean debounceAlliance = false;
    private int rBumpPressed = 1;

    private boolean debounceSide = false;
    private int lBumpPressed = 1;

    // Variable that keeps track of where in the loop you are
    int test = 0;

    @Override
    public void init() {
        this.bot = new Robot(this.hardwareMap, this.telemetry, true);

        // initialize the robot and the onboard gyro
        this.bot.initBot();
        initImu();
        distSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distSensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // when the gyro is calibrated print true to the screen
        telemetry.addData("is calibrated", imu.isGyroCalibrated());
        telemetry.update();

        // These variables 'debounce' the bumpers. They make sure the user can hold down
        //    the bumper for longer than a tick without the system repeatedly switching the value
        checkAlliance();
        checkSide();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        switch (test) {
            case 0:
                bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                test++;
                break;

            case 1:
                int target = bot.autonDrive(MovementEnum.BACKWARD, (int)(TICKS_PER_INCH * 20));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.drive(0.7, 0.7);

                if(target >= (int)(TICKS_PER_INCH * 20)){
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    test++;
                }

                break;

            case 2:
                // the amount to turn
                int turn = 90;

                // if the heading is at or greater than the target stop the bot
                if (bot.adjustHeading(turn, 0.8, imu)) {
                    bot.stop();
                    test++;
                }

                break;

            case 3:
                double stopDistance = 50;
                double dist = distSensor.getDistance(DistanceUnit.CM);
                telemetry.addData("Distance", dist);

                // Drive the bot forward until the distance sensor reads under a certain distance
                if (dist <= stopDistance * 1.5 && dist > stopDistance) {
                    bot.drive(0.5, 0.5);
                } else if (dist <= stopDistance) {
                    bot.stop();
                    test++;
                    break;
                } else
                    bot.drive(1.0, 1.0);
        }

        telemetry.addData("case", test);
        telemetry.addData("curr heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
    }

    public void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }

    private void checkAlliance(){
        // Makes sure that when a bumper is pressed it does not switch with every loop
        //    Lets the team set the alliance (red or blue) they are on
        if(gamepad2.right_bumper){
            if(!debounceAlliance && rBumpPressed == -1){
                telemetry.addData("Alliance", "RED");
                telemetry.update();
                rBumpPressed *= -1;
                debounceAlliance = true;
            } else if(!debounceAlliance && rBumpPressed == 1){
                telemetry.addData("Alliance", "BLUE");
                telemetry.update();
                rBumpPressed *= -1;
                debounceAlliance = true;
            } else{ debounceAlliance = false; }
        }
    }

    private void checkSide(){
        // Makes sure that when a bumper is pressed it does not switch with every loop
        //   Lets the team set the side (right or left) that the bot is on
        if(gamepad2.left_bumper){
            if(!debounceSide && lBumpPressed == -1){
                telemetry.addData("Side", "RIGHT");
                telemetry.update();
                lBumpPressed *= -1;
                debounceSide = true;
            } else if(!debounceSide && lBumpPressed == 1){
                telemetry.addData("Side", "LEFT");
                telemetry.update();
                lBumpPressed *= -1;
                debounceSide = true;
            } else{ debounceSide = false; }
        }
    }
}