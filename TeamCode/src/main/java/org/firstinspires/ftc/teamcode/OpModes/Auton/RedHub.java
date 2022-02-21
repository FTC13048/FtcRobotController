package org.firstinspires.ftc.teamcode.OpModes.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
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
    private ElapsedTime timer;
    private int one, two, three;

    public enum AutonState {
        PULLOUT, TURN, STRAFEHUB, LIFT, DRIVEHUB, DUMP,
        TURNWAREHOUSE, DRIVEWAREHOUSE
    }

    @Override
    public void init() {
        robot = new RobotSubsystems(new GamePadEx(gamepad1), new GamePadEx(gamepad2), hardwareMap, telemetry, true);
        robot.init();

        vision = new VisionWrapper(telemetry);
        this.one = 0;
        this.two = 0;
        this.three = 0;

        timer = new ElapsedTime();
    }

    @Override
    public void init_loop() { // init loop is always detection
        // Get current detection every loop
        this.level = this.vision.currentDetermination();
        if (this.level != null) { // add one the level variable of whatever is detected each loop
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
    public void start() { // set the lift level to whatever is detected in init loop
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
        switch (state) {
            case PULLOUT:
                this.vision.stop();
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.TURN;
                } else{
                    robot.driveTrain.driveDistanceSensor(robot.driveTrain.distSensorBack, 17.0, 0.5);
                }
                break;

            case TURN:
                if(robot.driveTrain.getState() == DriveTrain.DriveTrainState.IDLE){
                    robot.driveTrain.waitNext();
                    state = AutonState.STRAFEHUB;
                } else{
                    robot.driveTrain.adjustHeading(180, 0.4);
                }
                break;

            case STRAFEHUB:
                break;
            case LIFT:
                break;
            case DRIVEHUB:
                break;
            case DUMP:
                break;
            case TURNWAREHOUSE:
                break;
            case DRIVEWAREHOUSE:
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