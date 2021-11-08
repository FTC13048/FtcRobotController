package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class TFCamera {
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private int cameraPixelWidth = 0; // FIND OUT PIXEL WIDTH AND CHANGE

    private static final String VUFORIA_KEY = "ARzU3JD/////AAABmeac74tsC0F5u4/HaLL2p3RlQWFd9jHF" +
            "L1EDGkmDQbwR7SXcuqRD89Qh9ZQGpvORTUN90hpB0KOifhJuCZMVyrid08K8LNezho4whpoJGoYVdVfWby" +
            "bLTc+NrTZC1zWGnGCt7wNejXNBUPII8NcgwHbspujpBUJQeBThLJIyYV7ZJUHcTTt4x7NFTwPwSegvJ1zD" +
            "qLKKDgO6glC+CP7WR+QRIWPR0D85t4T5swcQ9WYd1bPwiDs75Gp4Gzea8QXPi6k5TZoohtBISydGsVpmLJ" +
            "4cbU6UtJ9XUztWFvYgkKoBNDwO1AkAhEFIBCv+08jzmP74VuauLcIO9eTvmLmwNCRIfxU8XC9NkYp73Q3p" +
            "4/WC";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public TFCamera(Telemetry tele, HardwareMap hmap){
        telemetry = tele;
        hardwareMap = hmap;
    }

    public void initCamera() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }
    }

    public void findTargetLevel(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                if(updatedRecognitions.size() == 0){
                    telemetry.addData("No items detected", "Level 1");
                    telemetry.update();
                    stop();
                }

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;

                    // check label to see which target zone to go after.
                    if (recognition.getLabel().equals("Marker") && recognition.getRight() <
                            cameraPixelWidth/2) {
                        telemetry.addData("Target Zone", "2");
                        telemetry.update();
                        stop();
                    } else if (recognition.getLabel().equals("Marker") && recognition.getRight() >
                            cameraPixelWidth/2) {
                        telemetry.addData("Target Zone", "3");
                        telemetry.update();
                        stop();
                    } else {
                        telemetry.addData("Target Zone", "UNKNOWN");
                        telemetry.update();
                        stop();
                    }
                }
            }
        }
    }


    // ---------------------------------------------------------------------------------------
    //                                     HELPER METHODS
    // ---------------------------------------------------------------------------------------

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    private void stop(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
