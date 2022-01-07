package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DuckSpinner {
    private DcMotor duckSpinner;
    private final double DUCK_POWER = 0.7;

    public DuckSpinner(HardwareMap hmap, Telemetry tele){
        duckSpinner = hmap.get(DcMotor.class, "duckSpinner");

        duckSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tele.addData("Duck Spinner", "Initialized");
    }

    public void spinDuckRed(){
        duckSpinner.setPower(-DUCK_POWER);
    }

    public void spinDuckBlue(){
        duckSpinner.setPower(DUCK_POWER);
    }

    public void setDuckPower(double power){
        duckSpinner.setPower(power);
    }
}
