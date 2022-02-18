package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.DuckSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystems;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import java.util.ArrayList;

@TeleOp(name = "Tank Drive Subsystems", group = "TeleOp")
public class SubsystemTeleOp extends OpMode {
    private DuckSpinner duckSpinner;
    private Lift lift;
    private DriveTrain driveTrain;

    @Override
    public void init() {
        lift = new Lift(hardwareMap, telemetry, false);
        duckSpinner = new DuckSpinner(hardwareMap, telemetry, false);
        driveTrain = new DriveTrain(hardwareMap, telemetry, false);
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        driveTrain.updateTeleOpState(new GamePadEx(gamepad1), new GamePadEx(gamepad2));

        lift.updateTeleOpState(new GamePadEx(gamepad1), new GamePadEx(gamepad2));
        lift.updateState();

        duckSpinner.updateTeleOpState(new GamePadEx(gamepad1), new GamePadEx(gamepad2));
        duckSpinner.updateState();

        telemetry.update();
    }

    @Override
    public void stop(){
        driveTrain.stop();
        duckSpinner.stop();
        lift.stop();
    }
}
