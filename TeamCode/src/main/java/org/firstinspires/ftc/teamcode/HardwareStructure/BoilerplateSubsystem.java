package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BoilerplateSubsystem extends Subsystems {
    private DcMotor motor;
    private final double motorPower = 0.5;

    protected BoilerplateSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(telemetry);
        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Boilerplate Subsystem", "Initialized");
    }

    @Override
    public void updateState() {

    }

    @Override
    public void updateTeleopState(GamePadEx gp1, GamePadEx gp2) {

    }

    @Override
    public void stop() {

    }
}