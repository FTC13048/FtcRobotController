package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Hardware.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TankDrive", group="Teleop")

public class TankTele extends OpMode {

    private Robot bot = new Robot();

    // power percentage bot will be running at
    private static final double POWER_PERCENT = 0.75;
    private TankDrive tankDrive;

    @Override
    public void init() {
        // Initialize the bot and tank drive (this is done only once)
        bot.init(hardwareMap, telemetry);
        tankDrive = new TankDrive(bot);
    }

    @Override
    public void init_loop(){ }

    @Override
    public void start(){ }

    @Override
    public void loop() {
        // Anything in this loop is repeated over and over until it is stopped
        // All controls to drive bot are given to controller 1
        double leftStickY = (double) -gamepad1.left_stick_y;
        double rightTrigger = (double) gamepad1.right_trigger;
        double leftTrigger = (double) gamepad1.left_trigger;
        double rightStickY = (double) -gamepad1.right_stick_y;

        tankDrive.driveBot(leftStickY, rightStickY, rightTrigger, leftTrigger, POWER_PERCENT);
    }

    @Override
    public void stop(){ bot.stop(); }
}
