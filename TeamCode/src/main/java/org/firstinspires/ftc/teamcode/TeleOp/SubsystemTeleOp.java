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
    private DriveTrain driveTrain;
    private Lift lift;
    private DuckSpinner duckSpiner;

    @Override
    public void init() {
        driveTrain = new DriveTrain(hardwareMap, telemetry, false);
        lift = new Lift(hardwareMap, telemetry, false);
        duckSpiner =  new DuckSpinner(hardwareMap, telemetry, false);
    }

    @Override
    public void init_loop(){
        driveTrain.initLoopTeleOp();
        lift.initLoopTeleOp();
        duckSpiner.initLoopTeleOp();
    }

    @Override
    public void start() {
        driveTrain.startTeleOp();
        lift.startTeleOp();
        duckSpiner.startTeleOp();
    }

    @Override
    public void loop() {
        driveTrain.updateTeleOpState(new GamePadEx(gamepad1), new GamePadEx(gamepad2));

        lift.updateTeleOpState(new GamePadEx(gamepad1), new GamePadEx(gamepad2));
        lift.updateState();

        duckSpiner.updateTeleOpState(new GamePadEx(gamepad1), new GamePadEx(gamepad2));
        duckSpiner.updateState();
    }

    @Override
    public void stop(){
        driveTrain.stop();
        lift.stop();
        duckSpiner.stop();
    }
}
