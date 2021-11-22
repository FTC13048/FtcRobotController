package org.firstinspires.ftc.teamcode.VisionStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionStuff.GripPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class VisionTest extends OpMode {
    private VisionWrapper vision;

    @Override
    public void init() {
        vision.init(hardwareMap);
        vision.startStream();
    }

    @Override
    public void loop() {
        telemetry.addData("Current Level", vision.currentDetermination());
    }

    @Override
    public void stop() { vision.stop(); }
}
