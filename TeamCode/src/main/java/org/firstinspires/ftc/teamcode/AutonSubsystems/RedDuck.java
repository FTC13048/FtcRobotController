package org.firstinspires.ftc.teamcode.AutonSubsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystems;

@Autonomous(name="Red Duck Subsystems", group="Subsystems")
public class RedDuck extends OpMode {
    private RobotSubsystems robot;

    private RobotSubsystems.DetectedLevel level;
    private Lift.LiftLevel liftLevel;

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
        switch(state){
            case PULLOUT:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.DRIVEDUCK;
                } else{
                    robot.driveTrain.driveDistanceSensor(robot.driveTrain.distSensorBack, 17.0, 0.5);
                }
                break;

            case DRIVEDUCK:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    timer.reset();
                    state = AutonState.DUCK;
                } else{
                    robot.driveTrain.driveDistanceSensor(robot.driveTrain.distSensorLeft, 23.0, 0.4);
                }
                break;

            case DUCK:
                if(timer.seconds() >= 4){
                    robot.duckSpinner.stop();
                    robot.driveTrain.waitNext();
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
                    robot.driveTrain.driveDistanceSensor(robot.driveTrain.distSensorBack, 85.0, 0.4);
                    robot.lift.setTargetLevel(liftLevel);
                }
                break;

            case TURNHUB:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.DRIVEHUB;
                } else{
                    robot.driveTrain.adjustHeading(90, 0.4);
                }
                break;

            case DRIVEHUB:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.DUMP;
                } else{
                    robot.driveTrain.driveDistanceSensor(robot.driveTrain.distSensorBack, 12.0, 0.4);
                }
                break;

            case DUMP:
                if(robot.lift.getState() == Lift.LiftState.MOVEINTAKE){
                    robot.driveTrain.waitNext();
                    state = AutonState.DRIVESTORAGE;
                } else{
                    robot.lift.dump();
                }
                break;

            case DRIVESTORAGE:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.STRAFESTORAGE;
                } else{
                    robot.driveTrain.setTargetAndMove((int)(-RobotSubsystems.TICKS_PER_INCH * 40), -0.5);
                }
                break;

            case STRAFESTORAGE:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.DONE;
                } else{
                    robot.driveTrain.driveDistanceSensor(robot.driveTrain.distSensorLeft, 65.0, 0.4);
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
