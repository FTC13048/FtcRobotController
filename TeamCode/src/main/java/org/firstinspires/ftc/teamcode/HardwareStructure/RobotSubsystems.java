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

    // ------------------------------- INTAKE METHODS -------------------------------
    public void runIntake(){ intake.runIntake(); }

    public void runOuttake(){ intake.runOuttake(); }

    public void setIntakePower(double power){ intake.setIntakePower(power); }
}
