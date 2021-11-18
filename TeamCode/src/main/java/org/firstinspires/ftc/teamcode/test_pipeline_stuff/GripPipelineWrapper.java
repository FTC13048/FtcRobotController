package org.firstinspires.ftc.teamcode.test_pipeline_stuff;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class GripPipelineWrapper {
    // WEBCAM NOT CAMERA
    private final OpenCvWebcam webcam;
    private final ACustomPipeline pipeline;

    public GripPipelineWrapper(ACustomPipeline pipeline, HardwareMap hardwareMap) {
        this(pipeline, "Webcam 1", hardwareMap, 320, 240);
    }

    public GripPipelineWrapper(ACustomPipeline pipeline, String configName, HardwareMap hardwareMap, int cameraPixelWidth, int cameraPixelHeight) {
        // Get live-view port
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // WEBCAM NOT CAMERA
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(3000);
        this.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Start camera streaming
                webcam.startStreaming(cameraPixelWidth, cameraPixelHeight, OpenCvCameraRotation.UPRIGHT);
                webcam.setPipeline(pipeline);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened (pray never)
                 */
                throw new RuntimeException("CAMERA UNABLE TO BE OPENED NOT SURE WHY TRY OTHER CAM?");
            }
        });

        this.pipeline = pipeline;
    }

    public ElementPosition lastDetectedPosition() {
        return this.pipeline.lastDetectedPosition();
    }

    public void closeLiveview() {
        this.webcam.pauseViewport();
    }

    public float getCamFPS() {
        return this.webcam.getFps();
    }

    /*public void stop() {
        // TODO: unclear if this works
        this.webcam.closeCameraDeviceAsync(() -> {

        });
    }*/
}
