package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftNewBotUnused extends Subsystem {
    private DcMotor liftRight, liftLeft;
    private DcMotor intake;

    private LiftState liftState;
    private LiftLevel targetLevel;

    private ElapsedTime timer;
    private double intakePower;
    private double liftPower = 0.5;

    protected LiftNewBotUnused(HardwareMap hmap, Telemetry tele, boolean isAuton) {
        super(hmap, tele, isAuton);

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

        liftState = LiftState.INTAKE;
        targetLevel = LiftLevel.TOP;

        tele.addLine("Intake: Initialized");
        tele.addLine("Lift: Initialized");
    }

    @Override
    public void updateState() {
        switch (liftState) {
            case INTAKE:
                intake.setPower(intakePower);
                break;

            case MOVE:
                if (setLiftPos(targetLevel.numTicks)) {
                    liftState = LiftState.ATLEVEL;
                }
                break;

            case MOVEINTAKE:
                if (setLiftPos(LiftLevel.INTAKE.numTicks)) {
                    liftState = LiftState.INTAKE;
                }
                break;

            case ATLEVEL:
                liftLeft.setPower(0.0);
                liftRight.setPower(0.0);
                intake.setPower(0.0);
                break;

            case DUMP:
                intake.setPower(-1.0);
                if (timer.seconds() >= 3) {
                    liftState = LiftState.MOVEINTAKE;
                }
                break;

            case MANUAL:
                liftLeft.setPower(liftPower * 0.5);
                liftRight.setPower(liftPower * 0.5);
                intake.setPower(intakePower);
                break;
        }
    }

    @Override
    public void updateTeleOpState(GamePadEx gp1, GamePadEx gp2) {
        switch (liftState) {
            case INTAKE:
                if (gp2.getControlDown(GamePadEx.ControllerButton.LTRIGGER)) {
                    intakePower = gp2.getAxis(GamePadEx.ControllerAxis.LEFT_TRIGGER);
                } else if (gp2.getControlDown(GamePadEx.ControllerButton.RTRIGGER)) {
                    intakePower = -gp2.getAxis(GamePadEx.ControllerAxis.RIGHT_TRIGGER);
                } else {
                    intakePower = 0.0;
                }

                if (gp2.getControlDown(GamePadEx.ControllerButton.A)) {
                    liftState = LiftState.MOVE;
                }
                break;

            case MOVE:
                break;

            case MOVEINTAKE:
                break;

            case ATLEVEL:
                if (gp2.getControlDown(GamePadEx.ControllerButton.X)) {
                    liftState = LiftState.DUMP;
                }
                break;
            case DUMP:
                if (gp2.getControl(GamePadEx.ControllerButton.X)) {
                    timer.reset();
                }
                break;

            case MANUAL:
                if (gp2.getControlDown(GamePadEx.ControllerButton.LTRIGGER)) {
                    intakePower = gp2.getAxis(GamePadEx.ControllerAxis.LEFT_TRIGGER);
                } else if (gp2.getControlDown(GamePadEx.ControllerButton.RTRIGGER)) {
                    intakePower = -gp2.getAxis(GamePadEx.ControllerAxis.RIGHT_TRIGGER);
                } else {
                    intakePower = 0.0;
                }

                liftPower = gp2.getAxis(GamePadEx.ControllerAxis.RIGHT_Y);
                break;
        }

        if (gp2.getControlDown(GamePadEx.ControllerButton.BACK) && liftState != LiftState.MANUAL) {
            liftState = LiftState.MOVEINTAKE;
        }

        if (gp2.getControlDown(GamePadEx.ControllerButton.GUIDE)) {
            if (liftState != LiftState.MANUAL) {
                liftState = LiftState.MANUAL;
            } else {
                liftState = LiftState.MOVEINTAKE;
            }
        }

        if (gp2.getControlDown(GamePadEx.ControllerButton.B)) {
            switch (targetLevel) {
                case TOP:
                    targetLevel = LiftLevel.BOT;
                    break;

                case MID:
                    targetLevel = LiftLevel.TOP;
                    break;

                case BOT:
                    targetLevel = LiftLevel.MID;
                    break;
            }
        }

        telemetry.addData("Current State", liftState.state);
        telemetry.addData("Current Target Level", targetLevel.level);
    }

    @Override
    public void stop() {
        liftRight.setPower(0.0);
        liftRight.setPower(0.0);
        intake.setPower(0.0);
    }

    public boolean setLiftPos(int numTicks) {
        liftLeft.setTargetPosition(numTicks);
        liftRight.setTargetPosition(numTicks);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(liftPower);
        liftRight.setPower(liftPower);

        if (Math.abs(liftLeft.getCurrentPosition() - numTicks) <= 5) {
            liftLeft.setPower(0.0);
            liftRight.setPower(0.0);

            liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return true;
        }

        return false;
    }

    public void intake(double power) {
        if (Math.abs(LiftLevel.INTAKE.numTicks - liftLeft.getCurrentPosition()) <= 5 ||
                liftState == LiftState.INTAKE) {

            liftState = LiftState.INTAKE;
            intake.setPower(power);
        }
    }

    public enum LiftState {
        INTAKE("Intake"), MOVE("Move to Level"), ATLEVEL("At Level"), DUMP("Dump"),
        MOVEINTAKE("Move to Intake"), MANUAL("Manual Control");

        private String state;

        private LiftState(String state) {
            this.state = state;
        }

        public String getState() {
            return this.state;
        }
    }

    public enum LiftLevel {
        TOP(0, "Top"), MID(0, "Middle"),
        BOT(0, "Bottom"), INTAKE(0, "Intake");

        private int numTicks;
        private String level;

        private LiftLevel(int ticks, String level) {
            numTicks = ticks;
            this.level = level;
        }

        public int getTargetTicks() {
            return this.numTicks;
        }

        public String getLevel() {
            return this.level;
        }
    }
}
