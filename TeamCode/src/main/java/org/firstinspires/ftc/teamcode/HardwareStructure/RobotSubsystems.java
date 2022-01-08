package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotSubsystems {
    private CargoDropper cargoFlipper;
    private DistanceSensor distSensorBack, distSensorRight, distSensorLeft;
    private DriveTrain driver;
    private DuckSpinner duckSpinner;
    private Intake intake;
    private LinearSlide linSlide;

    public RobotSubsystems(HardwareMap hmap, Telemetry tele, boolean isAuton){
        driver = new DriveTrain(hmap, tele, isAuton);
        cargoFlipper = new CargoDropper(hmap, tele);
        duckSpinner = new DuckSpinner(hmap, tele);
        intake = new Intake(hmap, tele);
        linSlide = new LinearSlide(hmap, tele);

        if(isAuton){
            distSensorBack = new DistanceSensor(hmap, tele, DistanceSensor.SensorName.BACK);
            distSensorLeft = new DistanceSensor(hmap, tele, DistanceSensor.SensorName.LEFT);
            distSensorRight = new DistanceSensor(hmap, tele, DistanceSensor.SensorName.RIGHT);
        }

        tele.addData("All subsystems", "Initialized");
    }

    public CargoDropper getCargoFlipper(){ return cargoFlipper; }

    public DriveTrain getDriveTrain(){ return driver; }

    public DuckSpinner getDuckSpinner(){ return duckSpinner; }

    public Intake getIntake(){ return intake; }

    public LinearSlide getLinSlide(){ return linSlide; }

    public DistanceSensor getDistSensorBack(){ return distSensorBack; }

    public DistanceSensor getDistSensorRight(){ return distSensorRight; }

    public DistanceSensor getDistSensorLeft(){ return distSensorLeft; }
}
