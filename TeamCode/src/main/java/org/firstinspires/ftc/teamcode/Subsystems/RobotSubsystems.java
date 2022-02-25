package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotSubsystems {
    public DriveTrain driveTrain;
    public DuckSpinner duckSpinner;
    public Lift lift;
    public final GamePadEx gp1, gp2;

    public static final double TICKS_PER_REV = 403.9;
    public static final double TICKS_PER_INCH = TICKS_PER_REV / (4.0 * Math.PI);

    private final HardwareMap hmap;
    private final Telemetry tele;
    private boolean isAuton;

    public RobotSubsystems(GamePadEx gp1, GamePadEx gp2, HardwareMap map, Telemetry tele, boolean isAuton){
        hmap = map;
        this.tele = tele;
        this.isAuton = isAuton;

        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public void init(){
        lift = new Lift(hmap, tele, isAuton);
        driveTrain = new DriveTrain(hmap, tele, isAuton);
        duckSpinner = new DuckSpinner(hmap, tele, isAuton);
    }

    public void initLoopAuton(){
        driveTrain.initLoopAuton();
        lift.initLoopAuton();
        duckSpinner.initLoopAuton();
    }

    public void updateTeleOpState(){
        lift.updateTeleOpState(gp1, gp2);
        driveTrain.updateTeleOpState(gp1, gp2);
        duckSpinner.updateTeleOpState(gp1, gp2);
    }

    public void updateState(){
        lift.updateState();
        driveTrain.updateState();
        duckSpinner.updateState();
    }

    public void stop(){
        lift.stop();
        driveTrain.stop();
        duckSpinner.stop();
    }

    public enum DetectedLevel{
        TOP, MIDDLE, BOTTOM
    }
}
