package org.firstinspires.ftc.teamcode.VisionStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "vision with camera", group = "")

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
