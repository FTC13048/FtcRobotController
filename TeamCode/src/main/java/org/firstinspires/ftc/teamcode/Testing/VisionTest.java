package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VisionStuff.VisionWrapper;

@TeleOp(name = "vision test", group = "testing")

public class VisionTest extends OpMode {
    private VisionWrapper vision;

    @Override
    public void init() {
        vision = new VisionWrapper();
        vision.init(hardwareMap);
        telemetry.addData("status", "initialized");
    }

    @Override
    public void loop() {
        telemetry.addData("Current Level", vision.currentDetermination());
    }

    @Override
    public void stop() { vision.stop(); }
}
