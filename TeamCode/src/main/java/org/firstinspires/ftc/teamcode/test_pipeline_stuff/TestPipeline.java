package org.firstinspires.ftc.teamcode.test_pipeline_stuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.test_pipeline_stuff.GripPipelineWrapper;
import org.firstinspires.ftc.teamcode.test_pipeline_stuff.DetectBanana;

@TeleOp(name = "detect banananananana test", group = "")
public class TestPipeline extends OpMode {
    GripPipelineWrapper grip;

    @Override
    public void init() {
        // this pipeline does no resizing, instead we use webcam at aspect ratio 320:240\
        // Config name for webcam is "Webcam 1"
        grip = new GripPipelineWrapper(new DetectBanana(), hardwareMap);
    }


    @Override
    public void loop() {
        try {
            telemetry.addData("pos", grip.lastDetectedPosition());
            telemetry.addData("fps", grip.getCamFPS());
            telemetry.update();
        } catch (Exception e) {
            // if error tell us what
            telemetry.addLine(e.getMessage());
        }
    }
}
