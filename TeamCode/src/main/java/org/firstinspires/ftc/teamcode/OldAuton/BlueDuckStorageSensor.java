package org.firstinspires.ftc.teamcode.OldAuton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OldHardware.MovementEnum;
import org.firstinspires.ftc.teamcode.OldHardware.Robot;
import org.firstinspires.ftc.teamcode.VisionStuff.VisionWrapper;

@Autonomous(name = "Blue Duck Storage Sensor", group = "Storage")

public class BlueDuckStorageSensor extends OpMode {
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
        try{
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
        } catch(NullPointerException e){
            telemetry.addLine(e.getStackTrace().toString());
        }
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

            telemetry.addLine("--------------------------------------");
            telemetry.addData("distance back", bot.getBackDistanceCM());
            telemetry.addData("distance right", bot.getRightDistanceCM());
            telemetry.addData("distance left", bot.getLeftDistanceCM());

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
                if (bot.driveBackDistanceSensor(18.0, 0.4, MovementEnum.FORWARD)) {
                    bot.stop();
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if(bot.delay(0.20)){ caseNum++; }
                }

                break;

            case 2: // Turn 90 degrees
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

            case 3: // Drive backward 32 inches
                int target = bot.autonDrive(MovementEnum.BACKWARD, (int) (TICKS_PER_INCH * 32));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.drive(0.5, 0.5);

                if (target >= (int) (TICKS_PER_INCH * 32)) {
                    bot.stop();
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    timer.reset();
                    caseNum++;
                }

                break;

            case 4: // Run the duck spinner for 4 seconds
                bot.runDuckSpinner(-0.7);

                if (timer.seconds() > 4) {
                    bot.runDuckSpinner(0.0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    caseNum++;
                }

                break;

            case 5: // Strafe right 47 inches
                if (bot.driveLeftDistanceSensor(96.0, 0.4, MovementEnum.LEFTSTRAFE)) {
                    bot.stop();
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if(bot.delay(0.20)){ caseNum++; }
                }

                break;

            case 6: // Turn 270 degrees
                turn = 270;

                // if the heading is at or greater than the target stop the bot
                if (bot.adjustHeading(turn, 0.5, imu)) {
                    bot.stop();
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    caseNum++;
                }

                break;

            case 7: // Extend the liner slide
                if (this.level == VisionWrapper.DetectionLevel.LEVEL_ONE) {
                    bot.linSlide.setTargetPosition(bot.FIRST_LEVEL);

                    if (bot.linSlide.getCurrentPosition() <= bot.FIRST_LEVEL) {
                        bot.linSlide.setPower(0.0);
                        bot.stop();
                        bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        timer.reset();
                        caseNum++;
                    }
                }

                if (this.level == VisionWrapper.DetectionLevel.LEVEL_TWO) {
                    bot.linSlide.setTargetPosition(bot.SECOND_LEVEL);

                    if (bot.linSlide.getCurrentPosition() <= bot.SECOND_LEVEL) {
                        bot.linSlide.setPower(0.0);
                        bot.stop();
                        bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        timer.reset();
                        caseNum++;
                    }
                }

                if (this.level == VisionWrapper.DetectionLevel.LEVEL_THREE) {
                    bot.linSlide.setTargetPosition(bot.THIRD_LEVEL);

                    if (bot.linSlide.getCurrentPosition() <= bot.THIRD_LEVEL) {
                        bot.linSlide.setPower(0.0);
                        bot.stop();
                        bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        timer.reset();
                        caseNum++;
                    }
                }

                bot.linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.linSlide.setPower(0.5);

                break;

            case 8:
                telemetry.addData("case", "8");
                telemetry.addData("cargo pos", bot.cargoFlipper.getPosition());
                bot.cargoFlipper.setPosition(0.6);

                if (timer.seconds() > 3) {
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    caseNum++;
                }

                break;

            case 9: // Drive backward to the hub
                if (bot.driveBackDistanceSensor(8.0, 0.4, MovementEnum.FORWARD)) {
                    bot.stop();
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    timer.reset();
                    caseNum++;
                }

                break;

            case 10: // Flip the basket
                telemetry.addData("case", "9");
                telemetry.addData("cargo pos", bot.cargoFlipper.getPosition());
                bot.cargoFlipper.setPosition(0.9);

                if (timer.seconds() > 2) {
                    bot.cargoFlipper.setPosition(0.1);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    caseNum++;
                }

                break;

            case 11: // Strafe right 16 inches
                target = bot.autonDrive(MovementEnum.FORWARD, (int) (TICKS_PER_INCH * 40));
                bot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.drive(0.5, 0.5);

                if (target >= (int) (TICKS_PER_INCH * 40)) {
                    bot.stop();
                    bot.autonDrive(MovementEnum.STOP, 0);
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    caseNum++;
                }

                break;

            case 12: // Drive forward 40 inches
                if (bot.driveRightDistanceSensor(62.0, 0.4, MovementEnum.RIGHTSTRAFE)) {
                    bot.stop();
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    caseNum++;
                }

                break;

            case 13: // Retract the linear slide and stop the bot
                bot.linSlide.setTargetPosition(0);
                bot.linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bot.linSlide.setPower(0.4);

                if (bot.linSlide.getCurrentPosition() >= 0) {
                    bot.linSlide.setPower(0.0);
                    bot.stop();
                    bot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    caseNum++;
                }

                break;
        }

        telemetry.addData("distance back", bot.getBackDistanceCM());
        telemetry.addData("distance right", bot.getRightDistanceCM());
        telemetry.addData("distance left", bot.getLeftDistanceCM());
        telemetry.update();
    }

    public void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }
}