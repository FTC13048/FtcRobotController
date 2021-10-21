package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

    int test = 1;

    // Variable to keep track of where in the auton the code is
    int auto = 0;

    @Override
    public void init() {
        this.bot = new Robot(this.hardwareMap, this.telemetry, true);

        // initialize the robot and the onboard gyro
        this.bot.initBot();
        initImu();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // when the gyro is calibrated print true to the screen
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

                // if the heading is at or greater than the target stop the bot
                if (bot.adjustHeading(turn, 0.5, imu)) {
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
}