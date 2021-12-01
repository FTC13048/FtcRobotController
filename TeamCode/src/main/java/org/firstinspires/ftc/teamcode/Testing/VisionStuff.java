package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VisionStuff.VisionWrapper;

@TeleOp(name="tester", group="")
public class VisionStuff extends OpMode {
    private VisionWrapper vision;

    @Override
    public void init() {
        vision = new VisionWrapper();
        vision.init(hardwareMap);
    }

    @Override
    public void loop() {

    }
}
