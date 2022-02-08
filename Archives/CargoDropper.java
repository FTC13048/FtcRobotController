package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CargoDropper {
    private Servo cargoFlipper;
    private boolean servoRaised = false;
    private ElapsedTime timer;

    protected CargoDropper(HardwareMap hmap, Telemetry tele){
        cargoFlipper = hmap.get(Servo.class, "cargoFlipper");
        timer = new ElapsedTime();

        tele.addData("cargo flipper", "initialized");
    }

    public void setPos(double position){
        cargoFlipper.setPosition(position);
    }

    private void placeCargo(boolean pad2X){
        // if x is pressed, the arm can only switch position if the debounce variable if false
        //     if it is, then it will become true after the  position switches, allowing the
        //     player to hold down the button without it trying to switch positions
        if(pad2X){
            servoRaised = true;
            timer.reset();
        }

        if(servoRaised){
            cargoFlipper.setPosition(0.9);
            if(timer.seconds() >= 2){
                cargoFlipper.setPosition(0.1);
                servoRaised = false;
            }
        }
    }
}
