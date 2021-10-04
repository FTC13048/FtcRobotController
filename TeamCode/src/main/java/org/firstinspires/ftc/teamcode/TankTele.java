package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Hardware.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TankDrive", group="Teleop")

public class TankTele extends OpMode {

    private DcMotor FL, FR, BL, BR;

    @Override
    public void init() {
        initBot();
        telemetry.addData("Bot", "Initialized");
    }

    @Override
    public void init_loop(){ }

    @Override
    public void start() { }

    @Override
    public void loop() {
        double rightTrigger = gamepad1.right_trigger;
        double leftTrigger = gamepad1.left_trigger;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickY = gamepad1.right_stick_y;

        rightTrigger = Math.abs(rightTrigger) > 0.15 ? rightTrigger : 0.0;
        leftTrigger = Math.abs(leftTrigger) > 0.15 ? leftTrigger : 0.0;
        leftStickY = Math.abs(leftStickY) > 0.15 ? leftStickY : 0.0;
        rightStickY = Math.abs(rightStickY) > 0.15 ? rightStickY : 0.0;

        if(rightTrigger > 0.0){
            strafe(rightTrigger);
        } else if(leftTrigger > 0.0){
            strafe(leftTrigger);
        } else {
            FR.setPower(rightStickY);
            BR.setPower(rightStickY);
            FL.setPower(leftStickY);
            BL.setPower(leftStickY);
        }
    }

    @Override
    public void stop() {
        FL.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        BR.setPower(0.0);
    }

    private void initBot(){
        // Assign all motors to ports, name it this exact thing in the driver hub
        BR = hardwareMap.get(DcMotor.class, "back_right");
        FR = hardwareMap.get(DcMotor.class, "front_right");
        BL = hardwareMap.get(DcMotor.class, "back_left");
        FL = hardwareMap.get(DcMotor.class, "front_left");

        // Set motor direction according to their orientation on the bot
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Run all motors using encoders
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafe(double strafePower){
        BR.setPower(-strafePower);
        FR.setPower(strafePower);
        BL.setPower(strafePower);
        FL.setPower(-strafePower);
    }

    public void drive(double drivePower){
        BR.setPower(drivePower);
        FR.setPower(drivePower);
        BL.setPower(drivePower);
        FL.setPower(drivePower);
    }
}
