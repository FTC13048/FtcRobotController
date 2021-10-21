package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name="DistanceSensor", group="Autonomous")

public class DistanceAuton extends OpMode {
    private Robot bot;
    private ModernRoboticsI2cRangeSensor dist;

    @Override
    public void init() {
        bot = new Robot(hardwareMap, telemetry, true);
        bot.initBot();
        dist = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distSensor");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Distance", dist.getDistance(DistanceUnit.CM));
    }

    @Override
    public void loop() {

    }
}
