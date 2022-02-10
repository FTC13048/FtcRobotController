package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BoilerplateSubsystem extends Subsystem {
    private DcMotor motor;
    private final double motorPower = 0.5;

    protected BoilerplateSubsystem(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuton) {
        super(hardwareMap, telemetry, isAuton);
        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Boilerplate Subsystem: Initialized");
    }

    @Override
    public void updateState() {

    }

    @Override
    public void updateTeleOpState(GamePadEx gp1, GamePadEx gp2) {

    }

    @Override
    public void stop() {

    }
}