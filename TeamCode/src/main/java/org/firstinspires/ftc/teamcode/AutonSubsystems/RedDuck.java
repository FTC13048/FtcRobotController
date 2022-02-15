package org.firstinspires.ftc.teamcode.AutonSubsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystems;

@Autonomous(name="blue duck", group="subsystems")
public class RedDuck extends OpMode {
    private RobotSubsystems robot;
    private RobotSubsystems.DetectedLevel level;
    private AutonState state;
    private ElapsedTime timer;

    public enum AutonState{
        PULLOUT, DRIVEDUCK, DUCK,
        BACKTOHUB, TURNHUB, LIFTSLIDE,
        DRIVEHUB,
        DUMP, STRAFESTORAGE, DRIVESTORAGE, LOWERLIFT
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

    }

    @Override
    public void loop() {
        switch(state){
            case PULLOUT:
                robot.driveTrain.driveWithDistanceSensor(DriveTrain.Direction.NORTH, robot.driveTrain.distSensorBack, 17.0, 0.5);
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    state = AutonState.DRIVEDUCK;
                }
                break;

            case DRIVEDUCK:
                robot.driveTrain.driveWithDistanceSensor(DriveTrain.Direction.WEST, robot.driveTrain.distSensorLeft, 23.0, 0.5);
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    timer.reset();
                    state = AutonState.DUCK;
                }
                break;

            case DUCK:
                robot.duckSpinner.spinRed();
                if(timer.seconds() >= 4){
                    robot.duckSpinner.stop();
                    state = AutonState.BACKTOHUB;
                }
                break;

            case BACKTOHUB:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.TURNHUB;
                } else{
                    robot.driveTrain.driveWithDistanceSensor(DriveTrain.Direction.NORTH, robot.driveTrain.distSensorBack, 103.0, 0.5);
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


            case LIFTSLIDE:

                break;

            case DRIVEHUB:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    state = AutonState.DUMP;
                }
                robot.driveTrain.driveWithDistanceSensor(DriveTrain.Direction.SOUTH, robot.driveTrain.distSensorBack, 8.0, 0.5);
                break;

            case DUMP:
                break;
            case STRAFESTORAGE:
                break;
            case DRIVESTORAGE:
                break;
            case LOWERLIFT:
                break;
        }
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
