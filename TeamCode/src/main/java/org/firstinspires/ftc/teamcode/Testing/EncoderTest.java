package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name = "EncoderTest", group = "testing")

public class EncoderTest extends OpMode {
    private Robot bot;

    @Override
    // Initialize the robot (this is what happens when the play button is pressed)
    public void init() {
        bot = new Robot(hardwareMap, telemetry, true);
        bot.initBot();
        telemetry.addData("Bot", "Initialized");
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() { }

    @Override
    // loop code to be continuously cycled through after init
    public void loop() {
        bot.wheelTelem();
        telemetry.addData("slide pos", bot.linSlide.getCurrentPosition());
        telemetry.addData("intake left", bot.intakeLeft.getCurrentPosition());
        telemetry.addData("intake right", bot.intakeRight.getCurrentPosition());
        telemetry.addData("duck spinner", bot.duckSpinner.getCurrentPosition());

        telemetry.addData("distance back", bot.getBackDistanceCM());
        telemetry.addData("distance right", bot.getRightDistanceCM());
        telemetry.addData("distance left", bot.getLeftDistanceCM());
        telemetry.update();
    }

    @Override
    // Sets the power of all motors to 0 when stop button is pressed
    public void stop() {
        bot.drive(0.0, 0.0);
    }
}
