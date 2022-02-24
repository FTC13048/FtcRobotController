package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystems;
import org.firstinspires.ftc.teamcode.VisionStuff.VisionWrapper;

/**
 * A boilerplate auton OpMode that can be used to make new auton OpModes that use subsystems
 *
 */
//@Autonomous(name = "Subsystem Boilerplate Auton", group = "Subsystems") Uncomment this to make this OpMode useable
public class BoilerplateAutonOpMode extends OpMode {
    private RobotSubsystems robot;
    private VisionWrapper vision; // vision object for detection

    private RobotSubsystems.DetectedLevel level;
    private Lift.LiftLevel liftLevel;

    private AutonState state;
    private ElapsedTime timer;
    private int one, two, three;

    public enum AutonState {
        STATE1, STATE2, STATE3,
        DONE
    }

    @Override
    public void init() {
        robot = new RobotSubsystems(new GamePadEx(gamepad1), new GamePadEx(gamepad2), hardwareMap, telemetry, true);
        robot.init();

        vision = new VisionWrapper(telemetry);
        this.one = 0;
        this.two = 0;
        this.three = 0;

        timer = new ElapsedTime();
        state = AutonState.STATE1;
    }

    @Override
    public void init_loop() { // init loop is always detection
        // Get current detection every loop
        this.level = this.vision.currentDetermination();
        if (this.level != null) { // add one the level variable of whatever is detected each loop
            // Add to value if detected
            switch (this.level) {
                case BOTTOM:
                    this.one++;
                    break;
                case MIDDLE:
                    this.two++;
                    break;
                case TOP:
                    this.three++;
                    break;
            }

            telemetry.addData("Current detected level: ", this.level);

            telemetry.addLine("-------------------------------------");
            telemetry.addLine("Overall detection numbers: (PRESS A TO RESET)");
            telemetry.addData("LEVEL 1: ", this.one);
            telemetry.addData("LEVEL 2: ", this.two);
            telemetry.addData("LEVEL 3: ", this.three);

            telemetry.update();
        }
    }

    @Override
    public void start() { // set the lift level to whatever is detected in init loop
        robot.driveTrain.startAuton();
        switch(level){
            case TOP:
                liftLevel = Lift.LiftLevel.TOP;
                break;
            case MIDDLE:
                liftLevel = Lift.LiftLevel.MID;
                break;
            case BOTTOM:
                liftLevel = Lift.LiftLevel.BOT;
                break;
        }
    }

    @Override
    public void loop() {
        switch (state) {
            case STATE1:
                if (robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE) { // Switch to the next action if the bot is done moving
                    robot.driveTrain.waitForNext();
                    timer.reset(); // ALWAYS reset the timer before switching to a state that relies on a timer
                    state = AutonState.STATE2;
                } else {
                    // Perform STATE1 actions here
                }
                break;

            case STATE2:
                if (timer.seconds() >= 4) { // Switch to the next action when the timer is past 4 secondss
                    robot.driveTrain.waitForNext();
                    state = AutonState.STATE3;
                } else {
                    // Perform STATE2 actions here
                }
                break;

            case STATE3:
                if (false) { // Replace 'false' with the condition to end STATE3
                    robot.driveTrain.waitForNext();
                    state = AutonState.DONE;
                } else {
                    // Perform STATE3 actions here
                }
                break;

            case DONE: // End state. It stops the bot
                robot.stop();
                break;
        }

        robot.updateState();

        telemetry.addData("Auton state", state);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}