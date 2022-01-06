package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name = "TankDrive", group = "TeleOp")

public class TankTele extends OpMode {
    private Robot bot;
    private ElapsedTime timer;

    private boolean automateSlide = false;
    private boolean servoRaised = false;

    @Override
    // Initialize the robot (this is what happens when the play button is pressed)
    public void init() {
        bot = new Robot(hardwareMap, telemetry, false);
        bot.initBot();
        timer = new ElapsedTime();
        telemetry.addData("Bot", "Initialized");
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() { bot.start(); }

    @Override
    // loop code to be continuously cycled through after init
    public void loop() {
        telemetry.addData("haha lol alex go brrrrr", "");
//        leftChangePower();
//        rightChangePower();

        // get double numbers for each control to be inputted for the power functions of the bot
        double rightTrigger = gamepad1.right_trigger;
        double leftTrigger = gamepad1.left_trigger;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickY = gamepad1.right_stick_y;

        double intake = gamepad2.right_trigger;
        double outtake = gamepad2.left_trigger;
        double duckPower = gamepad2.left_stick_y;
        double slidePower = gamepad2.right_stick_y;

        // if the controls are not at least 15% pushed down, the bot will not move
        //     this is to account for drift in the hardware
        rightTrigger = Math.abs(rightTrigger) > 0.15 ? rightTrigger : 0.0;
        leftTrigger = Math.abs(leftTrigger) > 0.15 ? leftTrigger : 0.0;
        leftStickY = Math.abs(leftStickY) > 0.15 ? leftStickY : 0.0;
        rightStickY = Math.abs(rightStickY) > 0.15 ? rightStickY : 0.0;

        intake = intake > 0.15 ? intake : 0.0;
        duckPower = Math.abs(duckPower) > 0.15 ? duckPower : 0.0;
        slidePower = Math.abs(slidePower) > 0.15 ? slidePower : 0.0;

        updateServo();
        updateSlide(slidePower);
        updateDriving(rightTrigger, leftTrigger, leftStickY, rightStickY);
        updateSystem(intake, outtake, duckPower);
        //checkLevelThree();
        telemetry.update();
    }

    public void updateDriving(double rTrigger, double lTrigger, double lStickY, double rStickY){
        // if the right trigger is pressed down strafe at that power
        if (rTrigger > 0.0) {
            bot.strafe(rTrigger);
        } else if (lTrigger > 0.0) {
            // if the left trigger is pressed down, enter the negative value into the strafe
            //     function to make it strafe left
            bot.strafe(-lTrigger);
        } else {
            // Otherwise, set the power to whatever the Y sticks are
            bot.drive(rStickY, lStickY);
        }
    }

    public void updateSystem(double intake, double outtake, double duck){
        // if either the left or right triggers on gamepad two are pressed, run the
        //    intake at that power
        if(intake > 0.0){
            bot.runIntake(1.0);
        } else if(outtake > 0.0) {
                bot.runIntake(-1.0);
        } else{ bot.runIntake(0.0); }

        if(Math.abs(duck) > 0.0){ bot.runDuckSpinner(duck * 0.7); }
        else{ bot.runDuckSpinner(0.0); }
    }

    private void updateServo(){
        // if x is pressed, the arm can only switch position if the debounce variable if false
        //     if it is, then it will become true after the  position switches, allowing the
        //     player to hold down the button without it trying to switch positions
        if(gamepad2.x){
            servoRaised = true;
            timer.reset();
        }

        if(servoRaised){
            bot.cargoFlipper.setPosition(0.9);
            if(timer.seconds() >= 2){
                bot.cargoFlipper.setPosition(0.1);
                servoRaised = false;
            }
        }
    }

    private void updateSlide(double slide){
        // if x is pressed, the arm can only switch position if the debounce variable if false
        //     if it is, then it will become true after the  position switches, allowing the
        //     player to hold down the button without it trying to switch positions
        if(gamepad2.a){
            automateSlide = true;
        }

        if(automateSlide){
            // Arm goes to pushing position if it is at rest
            if(bot.setLinSlidePos(-500)){
                bot.cargoFlipper.setPosition(0.3);
                automateSlide = false;
            }
        }  else{
            if(Math.abs(slide) > 0.0){ bot.runLinSlide(slide * 0.5); }
            else{ bot.runLinSlide(0.0); }
            telemetry.addData("slide", bot.getLinSlidePos());
        }
    }

    @Override
    // Sets the power of all motors to 0 when stop button is pressed
    public void stop() {
        bot.drive(0.0, 0.0);
    }
}