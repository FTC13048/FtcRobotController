package org.firstinspires.ftc.teamcode.Hardware;

public class TankDrive {
    private Robot bot;

    // How much the button needs to be pressed down to activate (1 is the max)
    private static final double BUFFER_AREA = 0.15;

    public TankDrive(Robot robot){ bot = robot; }

    public void driveBot(double leftStickY, double rightStickY, double rightTrigger,
                         double leftTrigger, double powerPercentage){

        // If the right trigger is pressed down at least 15% strafe right
        if(rightTrigger > BUFFER_AREA){
            bot.strafe(powerPercentage * rightTrigger);
        } else if(leftTrigger > BUFFER_AREA){
            // If the left trigger is pressed down at least 15% strafe left (pass a negative
            //      value to the strafe method)
            bot.strafe(powerPercentage * -leftTrigger);
        } else if(Math.abs(leftStickY) > BUFFER_AREA || Math.abs(rightStickY) > BUFFER_AREA){
            // if either the right or left stick is moved at least 50% in either direction, set
            //      that power to each motor
            if (leftStickY > BUFFER_AREA || leftStickY < -BUFFER_AREA) {
                bot.BL.setPower(leftStickY);
                bot.FL.setPower(leftStickY);
            }
            if (rightStickY > BUFFER_AREA || rightStickY < -BUFFER_AREA) {
                bot.FR.setPower(rightStickY);
                bot.BR.setPower(rightStickY);
            }
        }

        else{
            // if no stick/trigger is pressed do nothing
            bot.BR.setPower(0.0);
            bot.FR.setPower(0.0);
            bot.FL.setPower(0.0);
            bot.BL.setPower(0.0);
        }
    }
}
