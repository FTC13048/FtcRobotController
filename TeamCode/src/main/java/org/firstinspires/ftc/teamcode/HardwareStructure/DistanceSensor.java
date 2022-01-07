package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    ModernRoboticsI2cRangeSensor distSensor;

    public DistanceSensor(HardwareMap hmap, Telemetry tele, SensorName name){
        if(name == SensorName.BACK){
            distSensor = hmap.get(ModernRoboticsI2cRangeSensor.class, "distSensorBack");
        }

        else if(name == SensorName.LEFT){
            distSensor = hmap.get(ModernRoboticsI2cRangeSensor.class, "distSensorLeft");
        }

        else{
            distSensor = hmap.get(ModernRoboticsI2cRangeSensor.class, "distSensorRight");
        }
    }

    public double getDistInches(){ return distSensor.getDistance(DistanceUnit.INCH); }

    public double getDistCM(){ return distSensor.getDistance(DistanceUnit.CM); }

    public static enum SensorName{
        BACK, LEFT, RIGHT;
    }
}
