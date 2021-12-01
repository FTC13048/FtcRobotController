package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.MiniPID;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

//@Autonomous(name = "PIDTester", group = "Autonomous")

public class PIDTester extends OpMode {
    // Figure out ticks per revolution and ticks per inch
    private static final double TICKS_PER_REV = 403.9 * (80.0/72.0);
    private static final double TICKS_PER_INCH = TICKS_PER_REV/(4.0 * Math.PI);

    // The robot, gyro (and parameters), and pid system needed to run the bot
    //   PID makes it so the robot can ease in and out of turns/driving so it becomes more accurate
    //     in its positioning (it is solely based on calculating error)
    private Robot bot;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private MiniPID pid;

    // These variables 'debounce' the bumpers. They make sure the user can hold down
    //    the bumper for longer than a tick without the system repeatedly switching the value
    private boolean debounceAlliance = false;
    private int rBumpPressed = 1;

    private boolean debounceSide = false;
    private int lBumpPressed = 1;

    // Variable used to loop through the auton cases
    int caseNum = 0;

    @Override
    public void init() {
        this.bot = new Robot(this.hardwareMap, this.telemetry, true);

        // initialize the robot and the onboard gyro
        this.bot.initBot();
        initImu();

        // Pass in the PID coefficients, set maximum error allowed to 1 of whatever unit, and
        //      scale the output so that it is between -1 and 1 (can be passed to motors)
        pid = new MiniPID(0.1, 0, 0);
        pid.setSetpointRange(1);
        pid.setOutputLimits(-1, 1);
        pid.setOutputRampRate(0.1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // when the gyro is calibrated print true to the screen
        telemetry.addData("is calibrated", imu.isGyroCalibrated());

        // set the side and alliance you are on
        checkAlliance();
        checkSide();
        telemetry.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        switch (caseNum) {
            case 0:
                // Amount bot needs to turn
                int turn = 180;

                // pass the amount needed to turn into the pid system and set the direction
                //   to false (meaning forward/regular direction) and move to next case
                pid.setSetpoint(turn);
                pid.setDirection(false);
                caseNum++;
                break;

            case 1:
                // Get the current heading of the bot and set it to a 0-359 scale by adding
                //    360 to it if it comes out less than 0 and print this to the driver hub
                float currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
                currHeading = currHeading < 0 ? 360 + currHeading : currHeading;
                telemetry.addData("Angle", currHeading);

                if(currHeading >= 180){
                    bot.stop();
                    caseNum++;
                    break;
                } else{
                    bot.turn(1);
                }

                telemetry.update();
                //break;

            case 2:
                // Get the current heading of the bot and set it to a 0-359 scale by adding
                //    360 to it if it comes out less than 0 and print this to the driver hub
                currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
                currHeading = currHeading < 0 ? 360 + currHeading : currHeading;
                telemetry.addData("Angle", currHeading);

                // Get the pid output from the current heading (will give a power for the motors
                //   to run at) and print this to the driver hub
                double pidVal = pid.getOutput(currHeading);
                telemetry.addData("pid value", pidVal);

                // if the angle is at the target (pid output is 0) stop turning and move to
                // the next case
                if (pid.getOutput() == 0) {
                    telemetry.addData("Moving from", "case 2");
                    bot.turn(0);
                    caseNum++;
                    break;
                }

                bot.turn(pidVal);
        }
    }

    private void initImu() {
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
                rBumpPressed *= -1;
            } else if(!debounceAlliance && rBumpPressed == 1){
                telemetry.addData("Alliance", "BLUE");
                telemetry.update();
                rBumpPressed *= -1;
            }

            debounceAlliance = true;
        } else{ debounceAlliance = false; }
    }

    private void checkSide(){
        // Makes sure that when a bumper is pressed it does not switch with every loop
        //   Lets the team set the side (right or left) that the bot is on
        if(gamepad2.left_bumper){
            if(!debounceSide && lBumpPressed == -1){
                telemetry.addData("Side", "RIGHT");
                lBumpPressed *= -1;
            } else if(!debounceSide && lBumpPressed == 1){
                telemetry.addData("Side", "LEFT");
                telemetry.update();
                lBumpPressed *= -1;
            }

            debounceSide = true;
        } else{ debounceSide = false; }
    }
}