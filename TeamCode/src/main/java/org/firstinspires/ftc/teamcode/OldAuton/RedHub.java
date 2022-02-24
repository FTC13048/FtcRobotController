package org.firstinspires.ftc.teamcode.OldAuton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.VisionStuff.VisionWrapper;

@Autonomous(name = "Red Hub No Duck", group = "Hub")

public class RedHub extends OpMode {
    // Figure out ticks per revolution and ticks per inch
    private static final double TICKS_PER_REV = 403.9;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (4.0 * Math.PI);

    // The bot, gyro (with parameters), and distance sensor objects
    private Robot bot;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private VisionWrapper vision;
    private VisionWrapper.DetectionLevel level;

    // Variable that keeps track of where in the loop you are
    private int caseNum = 0;
    private ElapsedTime timer;
    private int one, two, three;

    @Override
    public void init() {
        this.bot = new Robot(this.hardwareMap, this.telemetry, true);

        // initialize the robot and the onboard gyro
        this.bot.initBot();
        initImu();

        // initialize the ai object recognition
        vision = new VisionWrapper(telemetry);
        vision.init(hardwareMap);
        this.level = VisionWrapper.DetectionLevel.UNKNOWN; // immediately overwritten but safer without null
        this.one = 0;
        this.two = 0;
        this.three = 0;

        // initialize the timer
        timer = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Get current detection every loop
        this.level = this.vision.currentDeterminationOld();

        if (this.level != null) {
            // Add to value if detected
            switch (this.level) {
                case LEVEL_ONE:
                    this.one++;
                    break;
                case LEVEL_TWO:
                    this.two++;
                    break;
                case LEVEL_THREE:
                    this.three++;
                    break;
            }

            telemetry.addData("Current detected level: ", this.level);

            telemetry.addLine("-------------------------------------");
            telemetry.addLine("Overall detection numbers: (PRESS A TO RESET)");
            telemetry.addData("LEVEL 1: ", this.one);
            telemetry.addData("LEVEL 2: ", this.two);
            telemetry.addData("LEVEL 3: ", this.three);

            telemetry.update();
        }
    }

    @Override
    public void start() {
        bot.start();
    }

    @Override
    public void loop() {
        switch (caseNum) {
            case 0: // Stop openCV and set the motor modes
                this.vision.stop();
                bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                caseNum++;
                break;

            case 1: //  Drive forward 6 inches
                int target = bot.autonDrive(MovementEnum.FORWARD, (int) (TICKS_PER_INCH * 6));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.drive(0.5, 0.5);

                if (target >= (int) (TICKS_PER_INCH * 6)) {
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    caseNum++;
                }

                break;

            case 2: // Turn 90 degrees
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
                target = bot.autonDrive(MovementEnum.RIGHTSTRAFE, (int) (TICKS_PER_INCH * 25));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.drive(0.5, 0.5);

                if (target >= (int) (TICKS_PER_INCH * 25)) {
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    caseNum++;
                }

                break;

            case 4: // Extend the liner slide
                if (this.level == VisionWrapper.DetectionLevel.LEVEL_ONE) {
                    bot.linSlide.setTargetPosition(bot.FIRST_LEVEL);

                    if (bot.linSlide.getCurrentPosition() <= bot.FIRST_LEVEL) {
                        bot.linSlide.setPower(0.0);
                        bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        bot.stop();
                        timer.reset();
                        caseNum++;
                    }
                }

                if (this.level == VisionWrapper.DetectionLevel.LEVEL_TWO) {
                    bot.linSlide.setTargetPosition(bot.SECOND_LEVEL);

                    if (bot.linSlide.getCurrentPosition() <= bot.SECOND_LEVEL) {
                        bot.linSlide.setPower(0.0);
                        bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        bot.stop();
                        timer.reset();
                        caseNum++;
                    }
                }

                if (this.level == VisionWrapper.DetectionLevel.LEVEL_THREE) {
                    bot.linSlide.setTargetPosition(bot.THIRD_LEVEL);

                    if (bot.linSlide.getCurrentPosition() <= bot.THIRD_LEVEL) {
                        bot.linSlide.setPower(0.0);
                        bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        bot.stop();
                        timer.reset();
                        caseNum++;
                    }
                }

                bot.linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.linSlide.setPower(0.5);

                break;

            case 5:
                telemetry.addData("case", "5");
                telemetry.addData("cargo pos", bot.cargoFlipper.getPosition());
                bot.cargoFlipper.setPosition(0.4);

                if (timer.seconds() > 3) {
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    caseNum++;
                }

                break;

            case 6:
                bot.drive(0.5, 0.5);

                if(bot.getBackDistanceCM() <= 12.0){
                    bot.stop();
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    timer.reset();
                    caseNum++;
                }

                break;

            case 7: // Flip the basket
                telemetry.addData("case", "9");
                telemetry.addData("cargo pos", bot.cargoFlipper.getPosition());
                bot.cargoFlipper.setPosition(0.9);

                if (timer.seconds() > 3) {
                    bot.cargoFlipper.setPosition(0.1);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    caseNum++;
                }

                break;

            case 8: // Retract the linear slide and stop the bot
                bot.linSlide.setTargetPosition(0);
                bot.linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.linSlide.setPower(0.5);

                if (bot.linSlide.getCurrentPosition() >= 0) {
                    bot.linSlide.setPower(0.0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.stop();
                    caseNum++;
                }

                break;

            case 9:
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

            case 10:
                target = bot.autonDrive(MovementEnum.FORWARD, (int) (TICKS_PER_INCH * 70));
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

        telemetry.update();
    }

    public void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }
}