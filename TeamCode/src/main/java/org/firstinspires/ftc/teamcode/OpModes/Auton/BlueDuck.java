package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystems;

@Autonomous(name = "Blue Duck Subsystems", group = "Subsystems")
public class BlueDuck extends OpMode {
    private RobotSubsystems robot;

    private RobotSubsystems.DetectedLevel level;
    private Lift.LiftLevel liftLevel;

    private AutonState state;
    private ElapsedTime timer;

    public enum AutonState {
        PULLOUT, TURNDUCK, DRIVEDUCK, DUCK,
        STRAFEHUB, TURNHUB,
        DRIVEHUB,
        DUMP, DRIVESTORAGE, STRAFESTORAGE,
        DONE
    }

    @Override
    public void init() {
        robot = new RobotSubsystems(new GamePadEx(gamepad1), new GamePadEx(gamepad2), hardwareMap, telemetry, true);
        robot.init();

        timer = new ElapsedTime();
        state = AutonState.PULLOUT;
    }

    @Override
    public void init_loop() {
        level = RobotSubsystems.DetectedLevel.TOP;
    }

    @Override
    public void start() {
        robot.driveTrain.startAuton();
        switch (level) {
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
            case PULLOUT: // If ya know what i mean... ;)
                if (robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE) {
                    robot.driveTrain.waitNext();
                    state = AutonState.TURNDUCK;
                } else {
                    robot.driveTrain.driveDistanceSensor(robot.driveTrain.distSensorBack, 18.0, 0.4);
                }
                break;

            case TURNDUCK:
                if (robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE) {
                    robot.driveTrain.waitNext();
                    state = AutonState.DRIVEDUCK;
                } else {
                    robot.driveTrain.adjustHeading(90, 0.4);
                }
                break;

            case DRIVEDUCK:
                if (robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE) {
                    robot.driveTrain.waitNext();
                    timer.reset();
                    state = AutonState.DUCK;
                } else {
                    robot.driveTrain.setTargetAndMove((int) (-RobotSubsystems.TICKS_PER_INCH * 32), -0.5);
                }
                break;

            case DUCK:
                if (timer.seconds() >= 4) {
                    robot.duckSpinner.stop();
                    robot.driveTrain.waitNext();
                    state = AutonState.STRAFEHUB;
                } else {
                    robot.duckSpinner.spinBlue();
                }
                break;

            case STRAFEHUB:
                if (robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE) {
                    robot.driveTrain.waitNext();
                    state = AutonState.TURNHUB;
                } else {
                    robot.driveTrain.driveDistanceSensor(robot.driveTrain.distSensorLeft, 96.0, 0.4);
                    robot.lift.setTargetLevel(liftLevel);
                }
                break;

            case TURNHUB:
                if (robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE) {
                    robot.driveTrain.waitNext();
                    state = AutonState.DRIVEHUB;
                } else {
                    robot.driveTrain.adjustHeading(270, 0.4);
                }
                break;

            case DRIVEHUB:
                if (robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE) {
                    robot.driveTrain.waitNext();
                    state = AutonState.DUMP;
                } else {
                    robot.driveTrain.driveDistanceSensor(robot.driveTrain.distSensorBack, 12.0, 0.4);
                }
                break;

            case DUMP:
                if (robot.lift.getState() == Lift.LiftState.MOVEINTAKE) {
                    robot.driveTrain.waitNext();
                    state = AutonState.DRIVESTORAGE;
                } else {
                    robot.lift.dump();
                }
                break;

            case DRIVESTORAGE:
                if (robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE) {
                    robot.driveTrain.waitNext();
                    state = AutonState.STRAFESTORAGE;
                } else {
                    robot.driveTrain.setTargetAndMove((int) (-RobotSubsystems.TICKS_PER_INCH * 40), -0.5);
                }
                break;

            case STRAFESTORAGE:
                if (robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE) {
                    robot.driveTrain.waitNext();
                    state = AutonState.DONE;
                } else {
                    robot.driveTrain.driveDistanceSensor(robot.driveTrain.distSensorRight, 65.0, 0.4);
                }
                break;

            case DONE:
                robot.stop();
                break;
        }
    }
}
