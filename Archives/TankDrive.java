package org.firstinspires.ftc.teamcode.Archives;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Archives.Bot;
import org.firstinspires.ftc.teamcode.Subsystems.DirectionEnum;
import org.firstinspires.ftc.teamcode.Subsystems.DistanceSensor;

public class TankDrive {
    private Bot bot;

    // drives until distance sensor reads a certain distance and then returns true when there
    public boolean driveDistanceSensor(double stopDist, double power,
                                       DistanceSensor sensor, DirectionEnum movement) {
        // if the sensor reads the stop distance return true
        //    if it reads 5 inches within the stop distance, set the motor power to 6 times
        //    less than the entered power
        if (Math.abs(sensor.getDistCM() - stopDist) < 3) {
            stop();
            return true;
        } else {
            if (Math.abs(sensor.getDistCM() - stopDist) <= 15) {
                power /= 6;
            }

            switch (movement) {
                case FORWARD:
                    drive(power, power);
                    break;

                case BACKWARD:
                    drive(-power, -power);
                    break;

                case LEFTSTRAFE:
                    strafe(-power);
                    break;

                case RIGHTSTRAFE:
                    strafe(power);
                    break;
            }

            return false;
        }
    }

    // sets the target position of the robot given the direction and target position
    public int autonDrive(DirectionEnum movement, int target) {
        // will be current position of the highest encoder value
        int curPos = -1;
        switch (movement) {
            case FORWARD:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(-target);
                curPos = Math.max(-BR.getCurrentPosition(), Math.max(-BL.getCurrentPosition(), Math.max(FR.getCurrentPosition(), FL.getCurrentPosition())));
                break;

            case BACKWARD:
                FL.setTargetPosition(target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(target);
                curPos = Math.max(BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(-FR.getCurrentPosition(), -FL.getCurrentPosition())));
                break;

            case LEFTSTRAFE:
                FL.setTargetPosition(target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(target);
                curPos = Math.max(BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(FR.getCurrentPosition(), FL.getCurrentPosition())));
                break;

            case RIGHTSTRAFE:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(-target);
                curPos = Math.max(-BR.getCurrentPosition(), Math.max(-BL.getCurrentPosition(), Math.max(-FR.getCurrentPosition(), -FL.getCurrentPosition())));
                break;

            case STOP:
                FL.setTargetPosition(FL.getCurrentPosition());
                FR.setTargetPosition(FR.getCurrentPosition());
                BL.setTargetPosition(BL.getCurrentPosition());
                BR.setTargetPosition(BR.getCurrentPosition());
                break;
        }

        return curPos;
    }

    // Adjusts the heading of the bot using gyroscope, degree amount to turn and motor power
    public boolean adjustHeading(int degrees, double power) {
        // get the current heading of the bot (an angle from -180 to 180)
        float currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // to convert to a 0-360 scale, if the current heading is negative add
        //    360 to it
        currHeading = currHeading < 0 ? 360 + currHeading : currHeading;

        // difference between target and current heading
        double difference = degrees - currHeading;
        telemetry.addData("Difference is ", difference);

        // If the bot is within 30 degrees of the target, slow it down to 25% of the desired speed to prevent overshooting
        if (Math.abs(difference) <= 30) {
            turn(power / 6);
        } else { // Otherwise use normal speed
            turn(power);
        }

        // If the bot is within 1 degree of the target, stop the bot and return true
        if (Math.abs(difference) <= 0.5) {
            telemetry.addData("Stopping the ", "bot");
            stop();
            return true;
        }

        // return false otherwise
        return false;
    }
}