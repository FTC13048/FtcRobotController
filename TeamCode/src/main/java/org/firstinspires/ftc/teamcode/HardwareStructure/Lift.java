package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends Subsystems{
    private DcMotor liftRight, liftLeft;
    private DcMotor intake;

    private LiftStates liftState;
    private TargetHeight targetLevel;

    private ElapsedTime timer;
    private double intakePower;
    private final double liftPower = 0.5;

    protected Lift(HardwareMap hmap, Telemetry tele){
        super(tele);

        liftRight = hmap.get(DcMotor.class, "liftRight");
        liftLeft = hmap.get(DcMotor.class, "liftLeft");
        intake = hmap.get(DcMotor.class, "intake");

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new ElapsedTime();

        liftState = LiftStates.INTAKE;
        targetLevel = TargetHeight.TOP;
    }

    @Override
    public void updateState() {
        switch(liftState){
            case INTAKE:
                intake.setPower(intakePower);
                break;

            case MOVE:
                if(setLiftPos(targetLevel.numTicks)){
                    liftState = LiftStates.ATLEVEL;
                }
                break;

            case MOVEINTAKE:
                if(setLiftPos(TargetHeight.INTAKE.numTicks)){
                    liftState = LiftStates.INTAKE;
                }
                break;

            case ATLEVEL:
                liftLeft.setPower(0.0);
                liftRight.setPower(0.0);
                intake.setPower(0.0);
                break;

            case DUMP:
                intake.setPower(-1.0);
                if(timer.seconds() >= 3){
                    liftState = LiftStates.MOVEINTAKE;
                }
                break;
        }
    }

    @Override
    public void updateTeleopState(GamePadEx gp1, GamePadEx gp2) {
        switch(liftState){
            case INTAKE:
                intakePower = gp2.gamepad.left_stick_y;

                if(gp2.getControlDown(GamePadEx.ControllerButtons.A)){
                    liftState = LiftStates.MOVE;
                }
                break;

            case MOVE:
                break;

            case MOVEINTAKE:
                break;

            case ATLEVEL:
                if(gp2.getControlDown(GamePadEx.ControllerButtons.X)){
                    timer.reset();
                    liftState = LiftStates.DUMP;
                }
                break;
            case DUMP:
                if(gp2.gamepad.a){
                    timer.reset();
                }
                break;
        }

        if(gp2.getControlDown(GamePadEx.ControllerButtons.BACK)){
            liftState = LiftStates.MOVEINTAKE;
        }

        if(gp2.getControlDown(GamePadEx.ControllerButtons.B)){
            switch(targetLevel){
                case TOP:
                    targetLevel = TargetHeight.BOT;
                    break;

                case MID:
                    targetLevel = TargetHeight.TOP;
                    break;

                case BOT:
                    targetLevel = TargetHeight.MID;
                    break;
            }
        }
    }

    public void intake(double power){
        if(liftLeft.getCurrentPosition() >= (TargetHeight.INTAKE.numTicks - 5) ||
                liftState == LiftStates.INTAKE){

            liftState = LiftStates.INTAKE;
            intake.setPower(power);
        }
    }

    @Override
    public void stop() {
        liftRight.setPower(0.0);
        liftRight.setPower(0.0);
        intake.setPower(0.0);
    }

    public boolean setLiftPos(int numTicks){
        liftLeft.setTargetPosition(numTicks);
        liftRight.setTargetPosition(numTicks);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(liftPower);
        liftRight.setPower(liftPower);

        if(Math.abs(liftLeft.getCurrentPosition() - numTicks) <= 5){
            liftLeft.setPower(0.0);
            liftRight.setPower(0.0);

            liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return true;
        }

        return false;
    }

    public enum LiftStates{
        INTAKE, MOVE, ATLEVEL, DUMP, MOVEINTAKE
    }

    public enum TargetHeight{
        TOP(0), MID(0), BOT(0), INTAKE(0);

        private int numTicks;

        private TargetHeight(int ticks){
            numTicks = ticks;
        }

        public int getTargetTicks(){ return this.numTicks; }
    }
}
