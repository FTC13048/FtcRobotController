package org.firstinspires.ftc.teamcode.RobVisionStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    this.one = 0;
    this.two = 0;
    this.three = 0;
  }

  @Override
  public void init_loop() {

  }

  @Override
  public void start() {
    // start camera detection
    this.tensorflow.start();
  }


  @Override
  public void loop() {
    // Get current detection
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
