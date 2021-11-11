package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.TFCamera;

@Autonomous(name = "Red Side Duck", group = "Autonomous")
public class AutonRedDuck extends OpMode {
    // Figure out ticks per revolution and ticks per inch
    private static final double TICKS_PER_REV = 403.9;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (4.0 * Math.PI);

    // The bot, gyro (with parameters), and distance sensor objects
    private Robot bot;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private TFCamera camera;
    private ModernRoboticsI2cRangeSensor distSensor;
    // private ModernRoboticsI2cRangeSensor distSensor;

    // Variable that keeps track of where in the loop you are
    private int caseNum = 0;
    private ElapsedTime timer;

    @Override
    public void init() {
        this.bot = new Robot(this.hardwareMap, this.telemetry, true);

        // initialize the robot and the onboard gyro
        this.bot.initBot();
        initImu();
        camera = new TFCamera(telemetry, hardwareMap);
        camera.initCamera();
        distSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distSensor");

        // initialize the timer
        timer = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if(imu.isGyroCalibrated()){
            telemetry.addData("Gyro status", "calibrated");
        } else{
            telemetry.addData("Wait", "Gyro calibrating!");
        }

      telemetry.update();
    }

    @Override
    public void start() { }

    @Override
    public void loop() {
        switch (caseNum) {
            case 0:
                camera.findTargetLevel();
                bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                caseNum++;
                break;

            case 1:
                int target = bot.autonDrive(MovementEnum.BACKWARD, (int) (TICKS_PER_INCH * 10));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.drive(0.5, 0.5);

                if (target >= (int) (TICKS_PER_INCH * 8)) {
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    caseNum++;
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
                    caseNum++;
                }

                break;

            case 3:
                target = bot.autonDrive(MovementEnum.LEFTSTRAFE, (int) (TICKS_PER_INCH * 25));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.strafe(0.5);

                if (target >= (int) (TICKS_PER_INCH * 25)) {
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    timer.reset();
                    caseNum++;
                }

                break;

            case 4:
                bot.runDuckSpinner(0.5);

                if(timer.seconds() > 4){
                    bot.runDuckSpinner(0.0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    caseNum++;
                }

                break;

            case 5:
                target = bot.autonDrive(MovementEnum.FORWARD, (int) (TICKS_PER_INCH * 36));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.drive(0.5, 0.5);

                if (target >= (int) (TICKS_PER_INCH * 36)) {
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    caseNum++;
                }

                break;

            case 6:
                // the amount to turn
                turn = 270;

                // if the heading is at or greater than the target stop the bot
                if (bot.adjustHeading(turn, 0.5, imu)) {
                    bot.stop();
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    caseNum++;
                }

                break;

            case 7:
                target = bot.autonDrive(MovementEnum.BACKWARD, (int) (TICKS_PER_INCH * 36));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.drive(0.5, 0.5);

                if (target >= (int) (TICKS_PER_INCH * 36)) {
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    caseNum++;
                }

                break;

            case 8:
                bot.linSlide.setTargetPosition(bot.THIRD_LEVEL);
                bot.linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.linSlide.setPower(0.5);

                if(bot.linSlide.getCurrentPosition() <= bot.THIRD_LEVEL){
                    bot.linSlide.setPower(0.0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    timer.reset();
                    caseNum++;
                }

                break;

            case 9:
                telemetry.addData("case", "9");
                telemetry.addData("cargo pos", bot.cargoFlipper.getPosition());
                bot.cargoFlipper.setPosition(0.9);

                if(timer.seconds() > 2){
                    bot.cargoFlipper.setPosition(0.1);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    caseNum++;
                }

                break;

            case 10:
                target = bot.autonDrive(MovementEnum.RIGHTSTRAFE, (int) (TICKS_PER_INCH * 15));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.strafe(0.5);

                if (target >= (int) (TICKS_PER_INCH * 15)) {
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    caseNum++;
                }

                break;

            case 11:
                bot.drive(1.0, 1.0);

                if(distSensor.getDistance(DistanceUnit.CM) <= 30){
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    caseNum++;
                }

                break;

            case 12:
                bot.linSlide.setTargetPosition(0);
                bot.linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.linSlide.setPower(0.5);

                if(bot.linSlide.getCurrentPosition() >= 0){
                    bot.linSlide.setPower(0.0);
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