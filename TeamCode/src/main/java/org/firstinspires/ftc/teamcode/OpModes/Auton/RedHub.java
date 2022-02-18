package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystems;

/**
 * The red hub OpMode
 *
 */
@Autonomous(name = "Subsystem Boilerplate Auton", group = "Subsystems")
public class RedHub extends OpMode {
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
                if (robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE) {
                    robot.driveTrain.waitNext();
                    state = AutonState.STATE2;
                } else {

                }
                break;

            case STATE2:
                if (robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE) {
                    robot.driveTrain.waitNext();
                    timer.reset();
                    state = AutonState.STATE3;
                } else {

                }
                break;

            case STATE3:
                if (timer.seconds() >= 4) {
                    robot.driveTrain.waitNext();
                    state = AutonState.DONE;
                } else {

                }
                break;

            case DONE:
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