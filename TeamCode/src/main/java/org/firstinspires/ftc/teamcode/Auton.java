package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name= "gyro", group="Autonomous")
public class Auton extends OpMode {
    // Figure out ticks per revolution and ticks per inch
    private static final double TICKS_PER_REV = 0;
    private static final double TICKS_PER_INCH = TICKS_PER_REV/(4.0 * Math.PI);

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
    public void init_loop(){
        telemetry.addData("is calibrated", imu.isGyroCalibrated());
        telemetry.update();
    }

    @Override
    public void start() { }

    @Override
    public void loop() {
        switch(test){
            case 1:
                // the amount to turn
                int turn = 180;

                // if the heading is at or greater than the target stop the bot, reset the timer,
                //    and move to the next case
                if(adjustHeading(turn, 0.5)){
                    bot.stop();

                    // go in the opposite direction until the bot is at the heading then
                    //     stop the bot and move to the next case
                    if(adjustHeading(180, -0.25)){
                        bot.stop();
                        test++;
                        break;
                    }
                }
        }
    }

    public void initImu(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }

    public boolean adjustHeading(int degrees, double power){
        // get the current heading of the bot (an angle from -180 to 180)
        int currHeading = (int)imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

        // to convert to a 0-360 scale, if the current heading is negative add
        //    360 to it
        currHeading = currHeading < 0 ? 360+currHeading : currHeading;

        // give the turn angle and start turning
        int turn = degrees;
        double turnPower = power;
        bot.turn(turnPower);

        // if the turn power is positive (turning to the left) and the bot's heading is greater
        //     than or equal to the target stop and return true
        if(turnPower > 0 && Math.abs(currHeading) >= turn){
            bot.stop();
            return true;
        }

        // if the turn power is negative (turning to the right) and the bot's heading is less
        //    than or equal to the target return true
        else if(turnPower < 0 && Math.abs(currHeading) <= turn){
            bot.stop();
            return true;
        }

        // return false otherwise
        return false;
    }
}
