package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.LVMaxSonar;

@TeleOp(name="TESTING", group="")
public class Testing extends OpMode {
    private LVMaxSonar sensor;

    @Override
    public void init() {
        sensor = new LVMaxSonar(hardwareMap, telemetry);
    }

    @Override
    public void init_loop(){
        telemetry.addData("raw voltage", sensor.getRawVoltage());
        telemetry.addData("distance", sensor.getDistance());
    }

    @Override
    public void loop() {

    }
}
