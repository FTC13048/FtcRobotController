package org.firstinspires.ftc.teamcode.RobVisionStuff;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 * Determines level of Autonomous Bonus Level by viewing the right-most two positions for a Team
 * Shipping Element or Duck from the point of view of the driver square.
 *
 * FOR THIS TO WORK CAMERA MUST BE CENTERED ON TWO RIGHT SQUARES WITH THE
 * THIRD NOT IN VIEW. PlACE DUCK ON ANY OF THE THREE SQUARES IT SHOULD WORK
 *
 * NAME CAMERA IN CONFIG: "camera"
 */
public class TFWrapperRob {

  private static final String TFOD_MODEL_ASSET = "FreightFrenzy_DM.tflite";
  private static final String[] LABELS = {
      "Duck",
      //"Marker"  // NOTE: THIS IS THE TAPE ON THE FLOOR... NOT THE "TEAM SHIPPING ELEMENT"
      //       .... explains why wesley's code works at all LMAO it thinks it sees blue
      //       tape square but it is really unsure
  };

  private static final String VUFORIA_KEY = "ARzU3JD/////AAABmeac74tsC0F5u4/HaLL2p3RlQWFd9jHF" +
      "L1EDGkmDQbwR7SXcuqRD89Qh9ZQGpvORTUN90hpB0KOifhJuCZMVyrid08K8LNezho4whpoJGoYVdVfWby" +
      "bLTc+NrTZC1zWGnGCt7wNejXNBUPII8NcgwHbspujpBUJQeBThLJIyYV7ZJUHcTTt4x7NFTwPwSegvJ1zD" +
      "qLKKDgO6glC+CP7WR+QRIWPR0D85t4T5swcQ9WYd1bPwiDs75Gp4Gzea8QXPi6k5TZoohtBISydGsVpmLJ" +
      "4cbU6UtJ9XUztWFvYgkKoBNDwO1AkAhEFIBCv+08jzmP74VuauLcIO9eTvmLmwNCRIfxU8XC9NkYp73Q3p" +
      "4/WC";

  private VuforiaLocalizer vuforia;
  private TFObjectDetector tfod;
  private int numRemovedRecognitions;
  private int totalRecognitions;

  /**
   * Construct a TFWrapper object
   */
  public TFWrapperRob() {
    this.numRemovedRecognitions = 0;
    this.totalRecognitions = 0;
  }

  /**
   * Call in init
   *
   * @param hwMap hardwaremap instance from opmode
   */
  public void init(HardwareMap hwMap) {
    this.initVuforia(hwMap);
    this.initTfod(hwMap);
  }

  /**
   * Initialize the Vuforia localization engine.
   */
  private void initVuforia(HardwareMap hardwareMap) {
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    // MAKE SURE TO NAME CAMERA IN CONFIG AS BELOW OR CHANGE AS NEEDED
    parameters.cameraName = hardwareMap.get(WebcamName.class, "camera");

    vuforia = ClassFactory.getInstance().createVuforia(parameters);
  }

  /**
   * Initialize the TensorFlow Object Detection engine.
   */
  private void initTfod(HardwareMap hardwareMap) {
    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
        "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    tfodParameters.isModelTensorFlow2 = true;
    tfodParameters.minResultConfidence = 0.5f;
    tfodParameters.inputSize = 320; //higher value here should make more accurate but slower I think
    tfodParameters.maxNumDetections = 1; // Let's see if this removes noise or just sucks

    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
  }

  /**
   * Call when want to begin detection (Normally after pressing start, but could be in init_loop if
   * want to ensure camera is working before match)
   */
  public void start() {
    if (tfod != null) {
      tfod.activate();
      // The TensorFlow software will scale the input images from the camera to a lower resolution.
      // This can result in lower detection accuracy at longer distances (> 55cm or 22").
      // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
      // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
      // should be set to the value of the images used to create the TensorFlow Object Detection model
      // (typically 16/9).
      //tfod.setZoom(2.5, 16.0/9.0);
    }
  }

  /**
   * Update the internal determined level and return its value to caller
   *
   * @return The Bonus Level as determined by the network
   */
  public BonusLevel getDeterminedLevel() {
    return this.updateDetermination();
  }

  /**
   *
   * @return the number of recognitions removed for last update
   */
  public int getNumRemovedRecognitions() {
    return this.numRemovedRecognitions;
  }

  public int getTotalRecognitions() {
    return this.totalRecognitions;
  }

  /**
   * Do the thing!!! FOR THIS TO WORK CAMERA MUST BE CENTERED ON TWO RIGHT SQUARES WITH THE
   * THIRD NOT IN VIEW
   */
  private BonusLevel updateDetermination() {

    if (tfod != null) {

      // Initialize a list of all the current recognitions
      // (note: updated recognitions can be a null value if no change in recognitions)
      List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
      if (updatedRecognitions != null) {

        // REMOVE ALL THINGS THAT ARE PROBABLY NOT A DUCK OR TEAM SHIPPING ELEMENT
        // OPTIONAL STEP: IF # RECOGNITIONS ALWAYS GOOD THEN UNECESSARY
        this.numRemovedRecognitions = 0;
        this.totalRecognitions = updatedRecognitions.size();

        /*for (int i = updatedRecognitions.size() - 1; i >= 0; i--) {
          Recognition recognition = updatedRecognitions.get(i);
          if (recognition.getWidth() * recognition.getHeight() < 500
              || recognition.getWidth() - recognition.getHeight() > 50) {
            // If the difference between height and width is larger than 50, or the object is
            // very small, it's unlikely to be useful
            updatedRecognitions.remove(i); // So, remove the recognition
            this.numRemovedRecognitions++;
          }
        }*/

        // If nothing detected
        if (updatedRecognitions.size() == 0) {
          // Element is off-screen, so it's level 1
          return BonusLevel.LEVEL_THREE;
        }
        // If one object detected
        else if (updatedRecognitions.size() == 1) {
          Recognition recognition = updatedRecognitions.get(0);

          // If on the left half of camera view, level is 2 (middle position)
          if (recognition.getRight() - (recognition.getWidth() / 2.0) < recognition.getImageWidth() / 2.0) {
            return BonusLevel.LEVEL_ONE;
          }
          // Otherwise, on right half of screen, so level 3 (right position)
          else {
            return  BonusLevel.LEVEL_TWO;
          }
        }
        // If there are multiple objects still recognized in view
        else {
          float maxConfidence = 0;
          Recognition best = null;

          // Determine which has the highest confidence
          for (Recognition recognition : updatedRecognitions) {
            if (recognition.getConfidence() > maxConfidence) {
              maxConfidence = recognition.getConfidence();
              best = recognition;
            }
          }
          // should never happen but just in case i messed up set to unknown state
          if (best == null) {
            return BonusLevel.UNKNOWN;
          }
          // Determine as we did above which half the object is in
          else if (best.getRight() - (best.getWidth() / 2.0) < best.getImageWidth() / 2.0) {
            return BonusLevel.LEVEL_ONE;
          } else {
            return BonusLevel.LEVEL_TWO;
          }
        }
      }
    }

    return BonusLevel.UNKNOWN;
  }

  public void stop() {
    if (tfod != null) {
      tfod.shutdown();
    }
  }

  /**
   * Representation of autonomous bonus level
   */
  public enum BonusLevel {
    LEVEL_ONE("1 - Lowest Level"), LEVEL_TWO("2 - Middle Level"), LEVEL_THREE("3 - Top Level"), UNKNOWN("Unknown");

    private final String str;

    BonusLevel(String str) {
      this.str = str;
    }

    @Override
    public String toString() {
      return this.str;
    }
  }
}
