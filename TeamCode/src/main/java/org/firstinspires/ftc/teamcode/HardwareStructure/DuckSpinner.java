package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DuckSpinner extends Subsystems{
    private DcMotor duckSpinner;
    private final double DUCK_POWER = 0.7;

    protected DuckSpinner(HardwareMap hmap, Telemetry tele){
        super(tele);
        duckSpinner = hmap.get(DcMotor.class, "duckSpinner");

        duckSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
        duckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tele.addData("Duck Spinner", "Initialized");
    }

    public void spinDuckRed(){
        duckSpinner.setPower(DUCK_POWER);
    }

    public void spinDuckBlue(){
        duckSpinner.setPower(-DUCK_POWER);
    }

    public void setDuckPower(double power){
        duckSpinner.setPower(power);
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

    public enum SpinDirection{
        SPINRED(0.7),
        SPINBLUE(-0.7),
        STOP(0.0);

        private double power;

        private SpinDirection(double pow){
            power = pow;
        }

        public double getDuckPower(){
            return this.power;
        }
    }
}
