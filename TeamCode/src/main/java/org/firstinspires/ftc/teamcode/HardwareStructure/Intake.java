package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotor intakeLeft, intakeRight;

    protected Intake(HardwareMap hmap, Telemetry tele){
        intakeRight = hmap.get(DcMotor.class, "intakeRight");
        intakeLeft = hmap.get(DcMotor.class, "intakeLeft");

        // Set the direction of the system motors based on their orientation on the bot
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tele.addData("intake", "initialized");
    }

    // Runs the intake motors at a given power
    public void runIntake(double power) {
        intakeRight.setPower(Math.abs(power));
        intakeLeft.setPower(Math.abs(power));
    }

    // Outtakes at the given power
    public void runOuttake(double power){
        intakeRight.setPower(Math.abs(power) * -1);
        intakeLeft.setPower(Math.abs(power) * -1);
    }
}
