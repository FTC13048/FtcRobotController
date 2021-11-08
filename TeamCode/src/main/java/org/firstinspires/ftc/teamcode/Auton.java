package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "gyro", group = "Autonomous")
public class Auton extends OpMode {
    // Figure out ticks per revolution and ticks per inch
    private static final double TICKS_PER_REV = 403.9;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (4.0 * Math.PI);

    // The bot, gyro (with parameters), and distance sensor objects
    private Robot bot;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    // private ModernRoboticsI2cRangeSensor distSensor;

    // Variable that keeps track of where in the loop you are
    private int test = 0;
    private ElapsedTime timer;

    @Override
    public void init() {
        this.bot = new Robot(this.hardwareMap, this.telemetry, true);

        // initialize the robot and the onboard gyro
        this.bot.initBot();
        initImu();

        // initialize the timer
        timer = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // when the gyro is calibrated print true to the screen
        telemetry.addData("Gyro calibration status", imu.isGyroCalibrated());
        telemetry.update();
    }

    @Override
    public void start() { }

    @Override
    public void loop() {
        switch (test) {
            case 0:
                bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                test++;
                break;

            case 1:
                int target = bot.autonDrive(MovementEnum.BACKWARD, (int) (TICKS_PER_INCH * 8));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.drive(0.5, 0.5);

                if (target >= (int) (TICKS_PER_INCH * 8)) {
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    test++;
                }

                break;

            case 2:
                // the amount to turn
                int turn = 180;

                // if the heading is at or greater than the target stop the bot
                if (bot.adjustHeading(turn, 0.5, imu)) {
                    bot.stop();
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    test++;
                }

                break;

            case 3:
                target = bot.autonDrive(MovementEnum.RIGHTSTRAFE, (int) (TICKS_PER_INCH * 25));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.strafe(0.5);

                if (target >= (int) (TICKS_PER_INCH * 25)) {
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    test++;
                }

                break;
                
            case 4:
                target = bot.autonDrive(MovementEnum.FORWARD, (int)(TICKS_PER_INCH * 6));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.drive(0.2, 0.2);
                
                if(target >= (int)(TICKS_PER_INCH * 6)){
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    timer.reset();
                    test++;
                }
                
                break;

            case 5:
                bot.runDuckSpinner(0.3);

                if(timer.seconds() > 3){
                    bot.runDuckSpinner(0.0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    test++;
                }
        }
    }

    public void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }
}