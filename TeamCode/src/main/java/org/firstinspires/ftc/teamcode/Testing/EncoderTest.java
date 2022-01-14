package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name = "EncoderTest", group = "testing")

public class EncoderTest extends OpMode {
    private Robot bot;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;

    @Override
    // Initialize the robot (this is what happens when the play button is pressed)
    public void init() {
        bot = new Robot(hardwareMap, telemetry, true);
        bot.initBot();
        initImu();
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

        telemetry.addData("Gyro heading %.2f",
                imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

        telemetry.addData("distance back", bot.getBackDistanceCM());
        telemetry.addData("distance right", bot.getRightDistanceCM());
        telemetry.addData("distance left", bot.getLeftDistanceCM());
        telemetry.update();
    }

    private void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }

    @Override
    // Sets the power of all motors to 0 when stop button is pressed
    public void stop() {
        bot.drive(0.0, 0.0);
    }
}
