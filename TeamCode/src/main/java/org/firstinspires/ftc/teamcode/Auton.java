package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Hardware.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Autonomous")

public class Auton extends OpMode {
    // Figure out ticks per revolution and ticks per inch
    private static final double TICKS_PER_REV = 0;
    private static final double TICKS_PER_INCH = TICKS_PER_REV/(4.0 * Math.PI);

    Robot bot = new Robot();

    // Variable to keep track of where in the auton the code is
    int auto = 0;

    @Override
    public void init() {
        // initialize the robot
        bot.init(hardwareMap, telemetry);

        // when the motors are not powered, brake
        bot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void init_loop(){ }

    @Override
    public void start() { }

    @Override
    public void loop() {
        try {
            switch (auto) {
                case 0:
                    bot.changeRunModeAuton(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    telemetry.addData(">", bot.FL.getCurrentPosition());
                    telemetry.addData(">", auto);
                    telemetry.update();

                    auto++;
                    break;
                case 1:
                    // drive forward 70 inches (to the line) and stop (add all information to
                    //   telemetry)
                    int en = bot.autonDrive(MovementEnum.FORWARD, (int)(TICKS_PER_INCH * 10));
                    bot.changeRunModeAuton(DcMotor.RunMode.RUN_TO_POSITION);
                    bot.drive(0.5);
                    telemetry.addData("Cas2, en: ", en);
                    telemetry.addData("FL: ", bot.FL.getCurrentPosition());
                    telemetry.addData("FR: ", bot.FR.getCurrentPosition());
                    telemetry.addData("BL: ", bot.BL.getCurrentPosition());
                    telemetry.addData("BR: ", bot.BR.getCurrentPosition());

                    telemetry.update();

                    // stop when at 70 inches and move to next case. Reset encoders
                    if(en >= (int)(TICKS_PER_INCH * 70)){
                        bot.autonDrive(MovementEnum.STOP, 0);
                        bot.changeRunModeAuton(DcMotor.RunMode.RUN_USING_ENCODER);
                        bot.changeRunModeAuton(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        bot.strafe(0.0);
                        auto++;
                    }
                    break;
            }
        } catch(Exception e){

        }
    }
}
