package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TankDrive", group="Teleop")

public class TankTele extends OpMode {

    // The motors that will be used
    private DcMotor FL, FR, BL, BR;

    @Override
    // Initialize the robot (this is what happens when the play button is pressed)
    public void init() {
        initBot();
        telemetry.addData("Bot", "Initialized");
    }

    @Override
    public void init_loop(){ }

    @Override
    public void start() { }

    @Override
    // loop code to be continuously cycled through after init
    public void loop() {
        // get double numbers for each control to be inputed for the power functions of the bot
        double rightTrigger = gamepad1.right_trigger;
        double leftTrigger = gamepad1.left_trigger;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickY = gamepad1.right_stick_y;

        // if the controls are not at least 15% pushed down, the bot will not move
        //     this is to account for drift in the hardware
        rightTrigger = Math.abs(rightTrigger) > 0.15 ? rightTrigger : 0.0;
        leftTrigger = Math.abs(leftTrigger) > 0.15 ? leftTrigger : 0.0;
        leftStickY = Math.abs(leftStickY) > 0.15 ? leftStickY : 0.0;
        rightStickY = Math.abs(rightStickY) > 0.15 ? rightStickY : 0.0;

        // if the right trigger is pressed down strafe at that power
        if(rightTrigger > 0.0){
            strafe(rightTrigger);
        } else if(leftTrigger > 0.0){
            // if the left trigger is pressed down, enter the negative value into the strafe
            //     function to make it strafe left
            strafe(-leftTrigger);
        } else {
            // Otherwise, set the power to whatever the Y sticks are
            FR.setPower(rightStickY);
            BR.setPower(rightStickY);
            FL.setPower(leftStickY);
            BL.setPower(leftStickY);
        }
    }

    @Override
    // Sets the power of all motors to 0 when stop button is pressed
    public void stop() {
        FL.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        BR.setPower(0.0);
    }

    // initializes all key systems of the bot (motors, servos, etc.)
    private void initBot(){
        // Assign all motors to ports, name it this exact thing in the driver hub
        BR = hardwareMap.get(DcMotor.class, "back_right");
        FR = hardwareMap.get(DcMotor.class, "front_right");
        BL = hardwareMap.get(DcMotor.class, "back_left");
        FL = hardwareMap.get(DcMotor.class, "front_left");

        // Set motor direction according to their orientation on the bot
        //   motors on the left side will be reversed so that their directions coorespond to
        //      the motors on the right
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // Sets the strafing power for the robot (negative number will strafe left and positive
    //     will strafe right)
    public void strafe(double strafePower){
        BR.setPower(-strafePower);
        FR.setPower(strafePower);
        BL.setPower(strafePower);
        FL.setPower(-strafePower);
    }

    // Sets all motors to drive with the given power (positive forward negative backward)
    public void drive(double drivePower){
        BR.setPower(drivePower);
        FR.setPower(drivePower);
        BL.setPower(drivePower);
        FL.setPower(drivePower);
    }
}
