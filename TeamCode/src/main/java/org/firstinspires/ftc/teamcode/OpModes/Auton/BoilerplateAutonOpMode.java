package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystems;
import org.firstinspires.ftc.teamcode.VisionStuff.VisionWrapper;

@Autonomous(name="Red Duck Subsystems", group="Subsystems")
public class BoilerplateAutonOpMode extends OpMode {
    private RobotSubsystems robot;
    private VisionWrapper vision;

    private RobotSubsystems.DetectedLevel level;
    private Lift.LiftLevel liftLevel;

    private AutonState state;
    private ElapsedTime timer;
    private int one, two, three;

    public enum AutonState{
        STATE1, STATE2, STATE3
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
        state = AutonState.STATE1;
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
        // ACTION FOR STATE 1 HERE
    }

    @Override
    public void loop() {
        switch(state){
            case STATE1:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.STATE2;
                    // STATE2 ACTION
                }
                break;

            case STATE2:
                if(robot.driveTrain.readyForNext()){
                    robot.driveTrain.waitForNext();
                    state = AutonState.STATE3;
                    // ACTION FOR STATE3
                }
                break;

            case STATE3:
                if(timer.seconds() >= 4){

                }
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
