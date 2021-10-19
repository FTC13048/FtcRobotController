package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "gyro", group = "Autonomous")
public class Auton extends OpMode {
    // Figure out ticks per revolution and ticks per inch
    private static final double TICKS_PER_REV = 0;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (4.0 * Math.PI);

    Robot bot;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    ElapsedTime timer;

    int test = 1;

    // Variable to keep track of where in the auton the code is
    int auto = 0;

    @Override
    public void init() {
        this.bot = new Robot(this.hardwareMap, this.telemetry);
        // initialize the robot
        this.bot.initBot();
        initImu();
        timer = new ElapsedTime();
        timer.startTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("is calibrated", imu.isGyroCalibrated());
        telemetry.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        switch (test) {
            case 1:
                // the amount to turn
                int turn = 254;

                // if the heading is at or greater than the target stop the bot and reset the timer
                if (adjustHeading(turn, 0.5)) {
                    bot.stop();
                    test++;
                    break;
                }
        }
    }

    public void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }

    public boolean adjustHeading(int degrees, double power) {
        // get the current heading of the bot (an angle from -180 to 180)
        int currHeading = (int) imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

        // to convert to a 0-360 scale, if the current heading is negative add
        //    360 to it
        currHeading = currHeading < 0 ? 360 + currHeading : currHeading;
        // give the turn angle and start turning
        double difference = degrees - currHeading;
        telemetry.addData("Difference is ", difference);

        // If the bot is within 30 degrees of the target, slow it down to 25% of the desired speed to prevent overshooting
        if (difference <= 30) {
            bot.turn(0.0625);
        } else { // Otherwise use normal speed
            bot.turn(power);
        }

        // If the bot is within 2 degrees of the target, stop the bot
        if (difference <= 1) {
            telemetry.addData("Stopping the ", "bot");
            bot.stop();
            return true;
        }

        // return false otherwise
        return false;
    }
}