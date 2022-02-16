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
    private RobotSubsystems robot;

    @Override
    public void init() {
        robot = new RobotSubsystems(new GamePadEx(gamepad1), new GamePadEx(gamepad2), hardwareMap, telemetry, false);
        robot.init();
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robot.updateTeleOpState();
        robot.updateState();
    }

    @Override
    public void stop(){
        robot.stop();
    }
}
