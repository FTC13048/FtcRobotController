package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.RobVisionStuff.TFWrapperRob;

@Autonomous(name = "Blue Side USE", group = "Autonomous")
public class AutonBlue extends OpMode {
    // Figure out ticks per revolution and ticks per inch
    private static final double TICKS_PER_REV = 403.9;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (4.0 * Math.PI);

    // The bot, gyro (with parameters), and distance sensor objects
    private Robot bot;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private TFWrapperRob tensorFlow;
    private TFWrapperRob.BonusLevel bonusLevel;
    private ModernRoboticsI2cRangeSensor distSensor;
    // private ModernRoboticsI2cRangeSensor distSensor;

    // Variable that keeps track of where in the loop you are
    private int caseNum = 0;
    private ElapsedTime timer;
    private int one, two, three;
    private int targetLevel;

    @Override
    public void init() {
        bot = new Robot(hardwareMap, telemetry, true);
        this.bot.initBot();
        initImu();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {    }

    @Override
    public void loop() {
        switch(caseNum){
            case 0:
                bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                caseNum++;
                break;

            case 1:
                int target = bot.autonDrive(MovementEnum.BACKWARD, (int) (TICKS_PER_INCH * 20));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.drive(0.5, 0.5);

                if (target >= (int) (TICKS_PER_INCH * 20)) {
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    caseNum++;
                }

                break;

            case 2:
                // the amount to turn
                int turn = 90;

                // if the heading is at or greater than the target stop the bot
                if (bot.adjustHeading(turn, 0.5, imu)) {
                    bot.stop();
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    caseNum++;
                }

                break;

            case 3:
                target = bot.autonDrive(MovementEnum.BACKWARD, (int) (TICKS_PER_INCH * 70));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.drive(1.0, 1.0);

                if (target >= (int) (TICKS_PER_INCH * 70)) {
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    caseNum++;
                }

                break;
        }
    }

    public void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }
}