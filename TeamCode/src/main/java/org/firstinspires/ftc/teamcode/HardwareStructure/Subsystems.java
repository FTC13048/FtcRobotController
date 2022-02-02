package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Subsystems {
    protected final Telemetry telemetry;

    public Subsystems(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public abstract void updateState();
    public abstract void updateTeleopState(GamePadEx DrivingGP, GamePadEx OtherGP);

    public void initLoopTeleop() {}
    public void initLoopAuton() {}

    public void startTeleop() {}
    public void startAuton() {}

    public abstract void stop();
}