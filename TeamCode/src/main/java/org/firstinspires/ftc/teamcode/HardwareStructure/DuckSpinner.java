package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DuckSpinner extends Subsystems {
    private DcMotor duckSpinner;
    private final double DUCK_POWER = 0.7;
    private SpinDirection duckState;

    public DuckSpinner(HardwareMap hmap, Telemetry tele) {
        super(tele);
        duckSpinner = hmap.get(DcMotor.class, "duckSpinner");

        duckSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckState = SpinDirection.STOP;

        tele.addData("Duck Spinner", "Initialized");
    }

    @Override
    public void updateState() {
        duckSpinner.setPower(duckState.power);
    }

    @Override
    public void updateTeleopState(GamePadEx gp1, GamePadEx gp2) {
        if (gp2.gamepad.left_stick_y < -0.15) {
            duckState = SpinDirection.SPINBLUE;
        } else if (gp2.gamepad.left_stick_y > 0.15) {
            duckState = SpinDirection.SPINRED;
        } else {
            duckState = SpinDirection.STOP;
        }
    }

    public void spinRed() {
        duckState = SpinDirection.SPINRED;
    }

    public void spinBlue() {
        duckState = SpinDirection.SPINBLUE;
    }

    public SpinDirection getState() {
        return duckState;
    }

    @Override
    public void stop() {
        duckSpinner.setPower(SpinDirection.STOP.power);
    }

    public enum SpinDirection {
        SPINRED(0.7),
        SPINBLUE(-0.7),
        STOP(0.0);

        private double power;

        private SpinDirection(double pow) {
            power = pow;
        }

        public double getDuckPower() {
            return this.power;
        }
    }
}