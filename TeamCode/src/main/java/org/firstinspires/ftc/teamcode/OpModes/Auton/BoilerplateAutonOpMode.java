package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystems;

/**
 * A boilerplate auton OpMode that can be used to make new auton OpModes that use subsystems
 *
 */
//@Autonomous(name = "Subsystem Boilerplate Auton", group = "Subsystems") Uncomment this to make this OpMode useable
public class BoilerplateAutonOpMode extends OpMode {
    private RobotSubsystems robot;

    private AutonState state;
    private ElapsedTime timer;

    public enum AutonState {
        STATE1, STATE2, STATE3,
        DONE
    }

    @Override
    public void init() {
        robot = new RobotSubsystems(new GamePadEx(gamepad1), new GamePadEx(gamepad2), hardwareMap, telemetry, true);
        robot.init();

        timer = new ElapsedTime();
        state = AutonState.STATE1;
    }

    @Override
    public void init_loop() {
        // Run init loop code here
    }

    @Override
    public void start() {
        robot.driveTrain.startAuton();
    }

    @Override
    public void loop() {
        switch (state) {
            case STATE1:
                if (robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE) { // Switch to the next action if the bot is done moving
                    robot.driveTrain.waitNext();
                    timer.reset(); // ALWAYS reset the timer before switching to a state that relies on a timer
                    state = AutonState.STATE2;
                } else {
                    // Perform STATE1 actions here
                }
                break;

            case STATE2:
                if (timer.seconds() >= 4) { // Switch to the next action when the timer is past 4 secondss
                    robot.driveTrain.waitNext();
                    state = AutonState.STATE3;
                } else {
                    // Perform STATE2 actions here
                }
                break;

            case STATE3:
                if (false) { // Replace 'false' with the condition to end STATE3
                    robot.driveTrain.waitNext();
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