package org.firstinspires.ftc.teamcode.RobVisionStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobVisionStuff.TFWrapperRob.BonusLevel;

@TeleOp(name = "TEST DUCK VISION DETECTION ROP", group = "ROB")
//@Disabled
public class TestTensorFlowDuck extends OpMode {

  private TFWrapperRob tensorflow;
  private TFWrapperRob.BonusLevel bonusLevel;

  private int one, two, three;

  @Override
  public void init() {
    this.tensorflow = new TFWrapperRob();
    this.tensorflow.init(hardwareMap);

    this.bonusLevel = BonusLevel.UNKNOWN; // immediately overwritten but safer without null

    this.one = 0;
    this.two = 0;
    this.three = 0;

    telemetry.addLine("Initialized. :)");
    telemetry.update();
  }

  @Override
  public void init_loop() {
    telemetry.addLine("Nothing happens here press start after ~2 seconds.");
    telemetry.update();
  }

  @Override
  public void start() {
    // start camera detection
    this.tensorflow.start();
    telemetry.clear();
  }


  @Override
  public void loop() {
    // Get current detection every loop
    this.bonusLevel = this.tensorflow.getDeterminedLevel();

    if (this.bonusLevel != null) {
      // Add to value if detected
      switch (this.bonusLevel) {
        case LEVEL_ONE:
          this.one++;
          break;
        case LEVEL_TWO:
          this.two++;
          break;
        case LEVEL_THREE:
          this.three++;
          break;
      }
      telemetry.addData("Current detected level: ", this.bonusLevel);
      telemetry.addData("Number of removed recognitions this run: ", this.tensorflow.getNumRemovedRecognitions());
    }

    // Typically for auton we would sample the detector for 0.5-1.5 seconds
    // then use the most-polled output as our proper value
    // Here we just see all the outputs as telemetry for testing purposes

    // Reset values if desired
    if (gamepad1.a) {
      this.one = 0;
      this.two = 0;
      this.three = 0;
    }

    telemetry.addLine("-------------------------------------");
    telemetry.addLine("Overall detection numbers: (PRESS A TO RESET)");
    telemetry.addData("LEVEL 1: ", this.one);
    telemetry.addData("LEVEL 2: ", this.two);
    telemetry.addData("LEVEL 3: ", this.three);

    telemetry.update();
  }

  @Override
  public void stop() {
    this.tensorflow.stop();
  }
}
