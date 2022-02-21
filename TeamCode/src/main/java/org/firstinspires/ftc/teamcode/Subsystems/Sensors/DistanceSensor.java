package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    private ModernRoboticsI2cRangeSensor distSensor;
    private LVMaxSonar distSensorBack;
    private SensorName name;

    public DistanceSensor(HardwareMap hmap, Telemetry tele, SensorName name){
        this.name = name;
        if(name == SensorName.BACK){
            distSensor = hmap.get(ModernRoboticsI2cRangeSensor.class, "distSensorBack");
            tele.addData("Back Distance Sensor", "Initialized");
        }

        else if(name == SensorName.LEFT){
            distSensor = hmap.get(ModernRoboticsI2cRangeSensor.class, "distSensorLeft");
            tele.addData("Left Distance Sensor", "Initialized");
        }

        else if(name == SensorName.BACK){
            distSensor = hmap.get(ModernRoboticsI2cRangeSensor.class, "distSensorRight");
            tele.addData("Right Distance Sensor", "Initialized");
        }

        else{
            distSensorBack = new LVMaxSonar(hmap, tele);
        }
    }

    public double getDistInches(){ return distSensor.getDistance(DistanceUnit.INCH); }

    public double getDistCM(){
        if(name == SensorName.ANALOG){
            return distSensorBack.getDistance();
        }

        return distSensor.getDistance(DistanceUnit.CM);
    }

    public enum SensorName{
        BACK, LEFT, RIGHT, ANALOG
    }
}
