package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.DuckSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import java.util.ArrayList;

@TeleOp(name = "Tank Drive Subsystems", group = "TeleOp")
public class SubsystemTeleOp extends OpMode {
    private ArrayList<Subsystem> subsystemsToUse;

    @Override
    public void init() {
        subsystemsToUse = new ArrayList<Subsystem>();
        subsystemsToUse.add(new DriveTrain(hardwareMap, telemetry, false));
        subsystemsToUse.add(new Lift(hardwareMap, telemetry, false));
        subsystemsToUse.add(new DuckSpinner(hardwareMap, telemetry, false));
    }

    @Override
    public void init_loop(){
        for(Subsystem subsystem : subsystemsToUse){
            subsystem.initLoopTeleOp();
        }
    }

    @Override
    public void start() {
        for(Subsystem subsystem : subsystemsToUse){
            subsystem.startTeleOp();
        }
    }

    @Override
    public void loop() {
        for(Subsystem subsystem : subsystemsToUse){
            subsystem.updateTeleOpState(new GamePadEx(gamepad1), new GamePadEx(gamepad2));
            subsystem.updateState();
        }
    }

    @Override
    public void stop(){
        for(Subsystem subsystem : subsystemsToUse){
            subsystem.stop();
        }
    }
}
