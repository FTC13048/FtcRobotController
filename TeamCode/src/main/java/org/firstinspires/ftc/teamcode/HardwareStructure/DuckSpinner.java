package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DuckSpinner extends Subsystems {
    private DcMotor duckSpinner;
    private final double DUCK_POWER = 0.7;
    private DuckState duckState;

    public DuckSpinner(HardwareMap hmap, Telemetry tele) {
        super(tele);
        duckSpinner = hmap.get(DcMotor.class, "duckSpinner");

        duckSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckState = DuckState.STOP;

        tele.addLine("Duck Spinner: Initialized");
    }

    @Override
    public void updateState() {
        duckSpinner.setPower(duckState.power);
    }

    @Override
    public void updateTeleopState(GamePadEx gp1, GamePadEx gp2) {
        if (gp2.getAxis(GamePadEx.ControllerAxis.LEFT_Y) < -0.15) {
            duckState = DuckState.SPINBLUE;
        } else if (gp2.getAxis(GamePadEx.ControllerAxis.LEFT_Y) > 0.15) {
            duckState = DuckState.SPINRED;
        } else {
            duckState = DuckState.STOP;
        }
    }

    public void spinRed() {
        duckState = DuckState.SPINRED;
    }

    public void spinBlue() {
        duckState = DuckState.SPINBLUE;
    }

    public DuckState getState() {
        return duckState;
    }

    @Override
    public void stop() {
        duckSpinner.setPower(DuckState.STOP.power);
    }

    public enum DuckState {
        SPINRED(0.7),
        SPINBLUE(-0.7),
        STOP(0.0);

        private double power;

        private DuckState(double pow) {
            power = pow;
        }

        public double getDuckPower() {
            return this.power;
        }
    }
}