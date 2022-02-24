package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystems;
import org.firstinspires.ftc.teamcode.VisionStuff.VisionWrapper;

@Autonomous(name="Red Duck Subsystems", group="Subsystems")
public class RedDuck extends OpMode {
    private RobotSubsystems robot;
    private VisionWrapper vision;

    private RobotSubsystems.DetectedLevel level;
    private Lift.LiftLevel liftLevel;

    private AutonState state;
    private ElapsedTime timer;
    private int one, two, three;

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

        vision = new VisionWrapper(telemetry);
        this.level = RobotSubsystems.DetectedLevel.TOP; // immediately overwritten but safer without null
        this.one = 0;
        this.two = 0;
        this.three = 0;

        timer = new ElapsedTime();
        state = AutonState.PULLOUT;
    }

    @Override
    public void init_loop() {
        // Get current detection every loop
        this.level = this.vision.currentDetermination();
        if (this.level != null) {
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

        robot.driveTrain.setTargetAndDrive((int)(-RobotSubsystems.TICKS_PER_INCH * 6), 0.1);
    }

    @Override
    public void loop() {
        switch(state){
            case PULLOUT:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitNext();
                    state = AutonState.DRIVEDUCK;
                    robot.driveTrain.setTargetAndStrafe((int)(RobotSubsystems.TICKS_PER_INCH * 19), 0.3);
                }
                break;

            case DRIVEDUCK:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitNext();
                    timer.reset();
                    state = AutonState.DUCK;
                }
                break;

            case DUCK:
                if(timer.seconds() >= 4){
                    robot.duckSpinner.stop();
                    robot.driveTrain.waitNext();
                    state = AutonState.BACKTOHUB;
                    robot.driveTrain.setTargetAndDrive((int)(-RobotSubsystems.TICKS_PER_INCH * 45), 0.3);
                    robot.lift.setTargetLevel(liftLevel, 0.6);
                } else{
                    robot.duckSpinner.spinRed();
                }
                break;

            case BACKTOHUB:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitNext();
                    state = AutonState.TURNHUB;
                    robot.driveTrain.adjustHeading(90, 0.3);
                } else{

                }
                break;

            case TURNHUB:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitNext();
                    state = AutonState.DRIVEHUB;
                    robot.driveTrain.setTargetAndDrive((int)(RobotSubsystems.TICKS_PER_INCH * 33), 0.3);
                } else{

                }
                break;

            case DRIVEHUB:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitNext();
                    state = AutonState.DUMP;
                } else{

                }
                break;

            case DUMP:
                if(robot.lift.getState() == Lift.LiftState.MOVEINTAKE){
                    robot.driveTrain.waitNext();
                    state = AutonState.DRIVESTORAGE;
                    robot.driveTrain.setTargetAndDrive((int)(-RobotSubsystems.TICKS_PER_INCH * 40), 0.3);
                } else{
                    robot.lift.dump();
                }
                break;

            case DRIVESTORAGE:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitNext();
                    state = AutonState.STRAFESTORAGE;
                    robot.driveTrain.setTargetAndStrafe((int)(RobotSubsystems.TICKS_PER_INCH * 15), 0.3);
                } else{

                }
                break;

            case STRAFESTORAGE:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitNext();
                    state = AutonState.DONE;

                } else{

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
