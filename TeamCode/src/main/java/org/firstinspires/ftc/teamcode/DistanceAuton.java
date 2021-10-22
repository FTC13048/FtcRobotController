package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "DistanceSensor", group = "Autonomous")

public class DistanceAuton extends OpMode {
    private Robot bot;
    private ModernRoboticsI2cRangeSensor distSensor;
    int auto = 0;

    @Override
    public void init() {
        bot = new Robot(hardwareMap, telemetry, true);
        bot.initBot();
        distSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distSensor");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Distance", distSensor.getDistance(DistanceUnit.CM));
    }

    @Override
    public void loop() {
        switch(auto){
            case 0:


            case 1:
                double stopDistance = 50;
                double dist = distSensor.getDistance(DistanceUnit.CM);
                telemetry.addData("Distance", dist);

                // Drive the bot forward until the distance sensor reads under a certain distance
                if (dist <= stopDistance * 1.5 && dist > stopDistance) {
                    bot.drive(-0.5, -0.5);
                } else if (dist <= stopDistance) {
                    bot.stop();
                    auto++;
                    break;
                } else
                    bot.drive(-1.0, -1.0);
        }
    }
}