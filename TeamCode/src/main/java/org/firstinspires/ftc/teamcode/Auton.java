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

    int turnAngle = 180;

    @Override
    public void loop() {
        switch (test) {
            case 1:
                int currHeading = (int) imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
                currHeading = currHeading < 0 ? 360 + currHeading : currHeading;
                telemetry.addData("turn", currHeading);
                telemetry.addData("desiredAngle", turnAngle);

                if (turnAngle != 0)
                    bot.turn(0.5);

                if (Math.abs(currHeading) > turnAngle) {
                    timer.reset();

                    while (timer.seconds() < 1) {
                        bot.turn(-0.5);
                    }

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


//    public void adjustHeading(int degrees) {
//        i++;
//        int turnAngle = degrees;
//        int currHeading = (int) imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
//        currHeading = currHeading < 0 ? 360 + currHeading : currHeading;
//        telemetry.addData("iterationNumber: ", String.valueOf(i), "    desiredDirection: ", String.valueOf(degrees), "   turn", currHeading);
//
//        if (currHeading > turnAngle - 3) {
//            bot.turn(0.5);
//            telemetry.addData("turned ", "bot");
//        } else
//            bot.stop();

//    }
}