package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LVMaxSonar {
    private AnalogInput distSensor;

    private double currentVoltage;
    private final double MICRO_VOLTS_PER_INCH = 6.4;
    private final double VOLTS_PER_CM = 0.00251969;

    public LVMaxSonar(HardwareMap map, Telemetry tele){
        distSensor = map.get(AnalogInput.class, "analogSensorBack");
    }

    public double getRawVoltage(){ return distSensor.getVoltage(); }

    public double getDistance(){
        currentVoltage = distSensor.getVoltage();

        return currentVoltage/VOLTS_PER_CM;
    }
}
