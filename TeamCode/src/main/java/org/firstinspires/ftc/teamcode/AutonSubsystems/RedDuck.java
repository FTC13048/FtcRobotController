package org.firstinspires.ftc.teamcode.AutonSubsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystems;

@Autonomous(name="blue duck", group="subsystems")
public class RedDuck extends OpMode {
    private RobotSubsystems robot;
    private RobotSubsystems.DetectedLevel level;
    private AutonState state;
    private ElapsedTime timer;

    public enum AutonState{
        PULLOUT, DRIVEDUCK, DUCK,
        BACKTOHUB, TURNHUB,
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
        switch(level){
            case TOP:
                robot.lift.targetLevel = Lift.LiftLevel.TOP;
                break;
            case MIDDLE:
                robot.lift.targetLevel = Lift.LiftLevel.MID;
                break;
            case BOTTOM:
                robot.lift.targetLevel = Lift.LiftLevel.BOT;
                break;
        }
    }

    @Override
    public void loop() {
        switch(state){
            case PULLOUT:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.DRIVEDUCK;
                } else{
                    robot.driveTrain.driveWithDistanceSensor(DriveTrain.Direction.NORTH, robot.driveTrain.distSensorBack, 17.0, 0.5);
                }
                break;

            case DRIVEDUCK:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    timer.reset();
                    state = AutonState.DUCK;
                } else{
                    robot.driveTrain.driveWithDistanceSensor(DriveTrain.Direction.WEST, robot.driveTrain.distSensorLeft, 23.0, 0.5);
                }
                break;

            case DUCK:
                if(timer.seconds() >= 4){
                    robot.duckSpinner.stop();
                    state = AutonState.BACKTOHUB;
                } else{
                    robot.duckSpinner.spinRed();
                }
                break;

            case BACKTOHUB:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.TURNHUB;
                } else{
                    robot.driveTrain.driveWithDistanceSensor(DriveTrain.Direction.NORTH, robot.driveTrain.distSensorBack, 103.0, 0.5);
                    robot.lift.liftState = Lift.LiftState.MOVE;
                }
                break;

            case TURNHUB:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.DRIVEHUB;
                } else{
                    robot.driveTrain.adjustHeading(90, 0.5);
                }
                break;

            case DRIVEHUB:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.DUMP;
                } else{
                    robot.driveTrain.driveWithDistanceSensor(DriveTrain.Direction.SOUTH, robot.driveTrain.distSensorBack, 8.0, 0.5);
                }
                break;

            case DUMP:
                break;

            case DRIVESTORAGE:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.STRAFESTORAGE;
                } else{
                    robot.driveTrain.setTargetAndMove((int)(RobotSubsystems.TICKS_PER_INCH * 35), DriveTrain.Direction.NORTH);
                }
                break;

            case STRAFESTORAGE:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                } else{
                    robot.driveTrain.driveWithDistanceSensor(DriveTrain.Direction.WEST, robot.driveTrain.distSensorLeft, 65.0, 0.5);
                }
                break;

            case DONE:
                break;
        }
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
