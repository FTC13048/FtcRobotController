package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide {
    private DcMotor linSlide;

    protected LinearSlide(HardwareMap hmap, Telemetry tele){
        linSlide = hmap.get(DcMotor.class, "linSlide");

        linSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tele.addData("linear slide", "initialized");
    }

    public void runLinSlide(double power) {
        linSlide.setPower(power);
    }

    public int getLinSlidePos() {
        return linSlide.getCurrentPosition();
    }

    public boolean setLinSlidePos(int numTicks){
        linSlide.setTargetPosition(numTicks);
        linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runLinSlide(0.4);

        if(Math.abs(linSlide.getCurrentPosition() - numTicks) <= 5){
            runLinSlide(0.0);
            linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return true;
        }

        return false;
    }
}
