package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Gyro {
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Gyro(HardwareMap hmap, Telemetry tele){
        hardwareMap = hmap;
        telemetry = tele;

        imu = hmap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();

        imu.initialize(parameters);
    }
}
