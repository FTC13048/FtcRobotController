package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends Subsystem {
    private DcMotor linSlide;
    private DcMotor intakeLeft, intakeRight;
    private Servo cargoFlipper;

    private LiftState liftState;
    private LiftLevel targetLevel;
    private LiftLevel origLevel;

    private ElapsedTime timer;
    private double intakePower;
    private double liftPower = 0.5;

    public Lift(HardwareMap hmap, Telemetry tele, boolean isAuton) {
        super(hmap, tele, isAuton);

        // Assign all system motors (not drive motors) names in the hub
        linSlide = hmap.get(DcMotor.class, "linSlide");
        intakeLeft = hmap.get(DcMotor.class, "intakeLeft");
        intakeRight = hmap.get(DcMotor.class, "intakeRight");
        cargoFlipper = hmap.get(Servo.class, "cargoFlipper");

        // Set the direction of the system motors based on their orientation on the bot
        linSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the zero power behavior for all the system motors
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all system motors to run using encoders
        intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime();

        liftState = LiftState.INTAKE;
        targetLevel = LiftLevel.TOP;

        tele.addLine("Lift: Initialized");
        tele.addLine("Intake: Initialized");
    }

    @Override
    public void updateState() {
        switch (liftState) {
            case INTAKE:
                intakeLeft.setPower(intakePower);
                intakeRight.setPower(intakePower);
                cargoFlipper.setPosition(0.1);
                break;

            case MOVE:
                if (setLinSlidePos(targetLevel.numTicks)) {
                    targetLevel = origLevel;
                    liftState = LiftState.ATLEVEL;
                }
                break;

            case MOVEINTAKE:
                if (setLinSlidePos(LiftLevel.INTAKE.numTicks)) {
                    liftState = LiftState.INTAKE;
                }
                break;

            case ATLEVEL:
                linSlide.setPower(0.0);
                intakeLeft.setPower(0.0);
                intakeRight.setPower(0.0);
                cargoFlipper.setPosition(0.3);
                break;

            case DUMP:
                cargoFlipper.setPosition(0.9);
                if (timer.seconds() > 1.5) {
                    cargoFlipper.setPosition(0.1);
                    liftState = LiftState.MOVEINTAKE;
                }

                break;

            case MANUAL:
                linSlide.setPower(liftPower * 0.5);
                intakeLeft.setPower(intakePower);
                intakeRight.setPower(intakePower);
                break;
        }
    }

    @Override
    public void updateTeleOpState(GamePadEx gp1, GamePadEx gp2) {
        switch (liftState) {
            case INTAKE:
                if (gp2.getControl(GamePadEx.ControllerButton.LTRIGGER)) {
                    intakePower = gp2.getAxis(GamePadEx.ControllerAxis.LTRIGGER);
                } else if (gp2.getControl(GamePadEx.ControllerButton.RTRIGGER)) {
                    intakePower = -gp2.getAxis(GamePadEx.ControllerAxis.RTRIGGER);
                } else {
                    intakePower = 0.0;
                }

                if (gp2.getControlDown(GamePadEx.ControllerButton.A)) {
                    liftState = LiftState.MOVE;
                    origLevel = targetLevel;
                    targetLevel = LiftLevel.BOT;
                }
                break;

            case MOVE:
                break;

            case MOVEINTAKE:
                break;

            case ATLEVEL:
                if (gp2.getControlDown(GamePadEx.ControllerButton.X)) {
                    liftState = LiftState.MOVE;
                }
                if (gp2.getControlDown(GamePadEx.ControllerButton.Y)) {
                    timer.reset();
                    liftState = LiftState.DUMP;
                }
                break;

            case DUMP:
                if (gp2.getControl(GamePadEx.ControllerButton.Y)) {
                    timer.reset();
                }
                break;

            case MANUAL:
                if (gp2.getControl(GamePadEx.ControllerButton.LTRIGGER)) {
                    intakePower = gp2.getAxis(GamePadEx.ControllerAxis.LTRIGGER);
                } else if (gp2.getControl(GamePadEx.ControllerButton.RTRIGGER)) {
                    intakePower = -gp2.getAxis(GamePadEx.ControllerAxis.RTRIGGER);
                } else {
                    intakePower = 0.0;
                }

                liftPower = gp2.getAxis(GamePadEx.ControllerAxis.RIGHT_Y);
                break;
        }

        // Start automatically moving the intake
        if (gp2.getControlDown(GamePadEx.ControllerButton.BACK) && liftState != LiftState.MANUAL) {
            liftState = LiftState.MOVEINTAKE;
        }

        // Switch between manual and automatic mode
        if (gp2.getControlDown(GamePadEx.ControllerButton.GUIDE)) {
            if (liftState != LiftState.MANUAL) {
                liftState = LiftState.MANUAL;
            } else {
                liftState = LiftState.MOVEINTAKE;
            }
        }

        // Cycle between goal positions
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

        telemetry.addData("Lift Pos", linSlide.getCurrentPosition());
        telemetry.addData("Current State", liftState.state);
        telemetry.addData("Current Target Level", targetLevel.level);
    }

    @Override
    public void stop() {
        linSlide.setPower(0.0);
        intakeLeft.setPower(0.0);
        intakeRight.setPower(0.0);
    }

    public boolean setLinSlidePos(int numTicks) {
        linSlide.setTargetPosition(numTicks);
        linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linSlide.setPower(0.4);

        if (Math.abs(linSlide.getCurrentPosition() - numTicks) <= 5) {
            linSlide.setPower(0.0);
            linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return true;
        }

        return false;
    }

    public void intake(double power) {
        if (Math.abs(LiftLevel.INTAKE.numTicks - linSlide.getCurrentPosition()) <= 5 ||
                liftState == LiftState.INTAKE) {

            liftState = LiftState.INTAKE;
            intakeLeft.setPower(power);
            intakeRight.setPower(power);
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
        TOP(-1435, "Top"), MID(-800, "Middle"),
        BOT(-881, "Bottom"), INTAKE(0, "Intake");

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
