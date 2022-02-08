package org.firstinspires.ftc.teamcode.Archives;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.tfod.TFObjectDetectorImpl;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

import java.util.List;

/**
 * A class of Object Detection.
 */
public class ObjectDetection {
    // Variable for parameters of Vuforia's Object Detection.
    private VuforiaLocalizer.Parameters VuforiaParameters;

    // Variable for parameters of Tensorflow's Object Detection.
    private TFObjectDetector.Parameters TFParameters;

    // Variable for Vuforia's Object Detection.
    private VuforiaLocalizer vuforiaLocalizer;

    // Variable for Tensorflow's Object Detection.
    private TFObjectDetector objectDetector;

    /**
     * @param vuforiaParameters Parameters for Vuforia's Object Detection System.
     * @param tfParameters Parameters for Tensorflow's Object Detection System.
     */
    public ObjectDetection(VuforiaLocalizer.Parameters vuforiaParameters, TFObjectDetector.Parameters tfParameters) {
        // Assign parameters to parameter variables.
        this.VuforiaParameters = vuforiaParameters;
        this.TFParameters = tfParameters;

        // Create Vuforia's Object Detection System than Tensorflow's Object Detection System.
        this.vuforiaLocalizer = new VuforiaLocalizerImpl(VuforiaParameters);
        this.objectDetector = new TFObjectDetectorImpl(tfParameters, vuforiaLocalizer);
    }

    /**
     * Loads Tensorflow Lite model into the Object Detection System.
     *
     * @param file_name Path to the TFLite model.
     * @param labels Labels of objects in the model.
     */
    public void LoadModel(String file_name, String labels) {
        objectDetector.loadModelFromFile(file_name, labels);
    }

    /**
     * Activates the model.
     */
    public void activate() {
        objectDetector.activate();
    }

    /**
     * Deactivates the model.
     */
    public void deactivate() {
        objectDetector.deactivate();
    }

    /**
     * List of Objects Currently Detected.
     * @return List of recognition object.
     */
    public List<Recognition> GetRecognition() {
        return objectDetector.getRecognitions();
    }


}