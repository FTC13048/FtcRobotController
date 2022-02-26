package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystems;
import org.firstinspires.ftc.teamcode.VisionStuff.VisionWrapper;

@Autonomous(name="Blue Duck Subsystems", group="Subsystems")
public class BlueDuck extends OpMode {
    private RobotSubsystems robot;
    private VisionWrapper vision;

    private RobotSubsystems.DetectedLevel level;
    private Lift.LiftLevel liftLevel;

    private AutonState state;
    private ElapsedTime timer;
    private int one, two, three;

    public enum AutonState{
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

        vision = new VisionWrapper(telemetry);
        vision.init(hardwareMap);
        this.level = RobotSubsystems.DetectedLevel.TOP; // immediately overwritten but safer without null
        this.one = 0;
        this.two = 0;
        this.three = 0;

        timer = new ElapsedTime();
        timer.reset();
        state = AutonState.PULLOUT;
    }

    @Override
    public void init_loop() {
        try {
            if(timer.seconds() >= 5.0){
                this.one = 0;
                this.two = 0;
                this.three = 0;
                timer.reset();
            }
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
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void start() {
        robot.driveTrain.startAuton();
        if(Math.max(this.one, Math.max(this.two, this.three)) == this.one){
            liftLevel = Lift.LiftLevel.BOT;
        } else if(Math.max(this.one, Math.max(this.two, this.three)) == this.two){
            liftLevel = Lift.LiftLevel.MID;
        } else{
            liftLevel = Lift.LiftLevel.TOP;
        }

        robot.driveTrain.setTargetAndDrive((int)(-RobotSubsystems.TICKS_PER_INCH * 6), 0.1);
    }

    @Override
    public void loop() {
        switch(state){
            case PULLOUT:
                vision.stop();
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.TURNDUCK;
                    robot.driveTrain.adjustHeading(90, 0.2);
                } else{

                }
                break;

            case TURNDUCK:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.DRIVEDUCK;
                    robot.driveTrain.setTargetAndDrive((int)(RobotSubsystems.TICKS_PER_INCH * 34), 0.3);
                } else{

                }
                break;

            case DRIVEDUCK:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    timer.reset();
                    state = AutonState.DUCK;
                } else{

                }
                break;

            case DUCK:
                if(timer.seconds() >= 4){
                    robot.duckSpinner.stop();
                    robot.driveTrain.waitForNext();
                    state = AutonState.STRAFEHUB;
                    robot.driveTrain.setTargetAndStrafe((int)(-RobotSubsystems.TICKS_PER_INCH * 50), 0.3);
                    robot.lift.setTargetLevel(liftLevel, 0.6);
                } else{
                    robot.duckSpinner.spinBlue();
                }
                break;

            case STRAFEHUB:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.TURNHUB;
                    robot.driveTrain.adjustHeading(270, 0.2);
                } else{

                }
                break;

            case TURNHUB:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.DRIVEHUB;
                    robot.driveTrain.setTargetAndDrive((int)(RobotSubsystems.TICKS_PER_INCH * 32), 0.3);
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
                    state = AutonState.DRIVESTORAGE;
                    robot.driveTrain.setTargetAndDrive((int)(-RobotSubsystems.TICKS_PER_INCH * 40), 0.3);
                } else{
                    robot.lift.dump();
                }
                break;

            case DRIVESTORAGE:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.STRAFESTORAGE;
                    robot.driveTrain.setTargetAndStrafe((int)(-RobotSubsystems.TICKS_PER_INCH * 23), 0.3);
                } else{

                }
                break;

            case STRAFESTORAGE:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.DONE;
                } else{

                }
                break;

            case DONE:
                robot.stop();
                break;
        }

        robot.updateState();
        telemetry.addData("State", state);
    }
}
