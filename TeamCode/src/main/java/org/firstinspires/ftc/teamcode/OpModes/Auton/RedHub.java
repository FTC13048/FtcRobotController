package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystems;
import org.firstinspires.ftc.teamcode.VisionStuff.VisionWrapper;

/**
 * Auton that goes to the red hub (does not do ducks)
 *
 */
@Autonomous(name = "Red Hub Subsystems", group = "Subsystems")
public class RedHub extends OpMode {
    private RobotSubsystems robot;
    private VisionWrapper vision; // vision object for detection

    private RobotSubsystems.DetectedLevel level;
    private Lift.LiftLevel liftLevel;

    private AutonState state;
    private int one, two, three;

    public enum AutonState {
        PULLOUT, TURN, STRAFEHUB, LIFT, DRIVEHUB, DUMP,
        BACKWAREHOUSE, TURNWAREHOUSE, DRIVEWAREHOUSE, DONE
    }

    @Override
    public void init() {
        robot = new RobotSubsystems(new GamePadEx(gamepad1), new GamePadEx(gamepad2), hardwareMap, telemetry, true);
        robot.init();

        vision = new VisionWrapper(telemetry);
        vision.init(hardwareMap);
        this.level = RobotSubsystems.DetectedLevel.TOP; // immediately overwritten but safer without null
        this.one = 0;
        this.two = 0;
        this.three = 0;

        state = AutonState.PULLOUT;
    }

    @Override
    public void init_loop() {
        try {
            // Get current detection every loop
            robot.initLoopAuton();
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
        } catch (Exception e) {
            e.printStackTrace();
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
        switch (state) {
            case PULLOUT:
                vision.stop();
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.TURN;
                    robot.driveTrain.adjustHeading(180, 0.2);
                } else{

                }
                break;

            case TURN:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.STRAFEHUB;
                    robot.driveTrain.setTargetAndStrafe((int)(-RobotSubsystems.TICKS_PER_INCH * 25), 0.3);
                } else{

                }
                break;

            case STRAFEHUB:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.LIFT;
                    robot.lift.setTargetLevel(liftLevel, 0.6);
                } else{

                }
                break;

            case LIFT:
                if(robot.lift.getState() == Lift.LiftState.ATLEVEL){
                    robot.driveTrain.waitForNext();
                    state = AutonState.DRIVEHUB;
                    robot.driveTrain.setTargetAndDrive((int)(RobotSubsystems.TICKS_PER_INCH * 25), 0.3);
                } else{

                }
                break;

            case DRIVEHUB:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.DUMP;
                } else{

                }
                break;

            case DUMP:
                if(robot.lift.getState() == Lift.LiftState.MOVEINTAKE){
                    robot.driveTrain.waitForNext();
                    state = AutonState.BACKWAREHOUSE;
                    robot.driveTrain.setTargetAndDrive((int)(-RobotSubsystems.TICKS_PER_INCH * 10), 0.3);
                } else{
                    robot.lift.dump();
                }
                break;

            case BACKWAREHOUSE:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.TURNWAREHOUSE;
                    robot.driveTrain.adjustHeading(270, 0.2);
                }
                break;

            case TURNWAREHOUSE:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.DRIVEWAREHOUSE;
                    robot.driveTrain.setTargetAndDrive((int)(-RobotSubsystems.TICKS_PER_INCH * 70), 1.0);
                } else{

                }
                break;

            case DRIVEWAREHOUSE:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.DONE;
                } else{

                }
                break;

            case DONE:
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