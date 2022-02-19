package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A subsystem. Subsystems are used to add new behaviors to auton or teleop.
 * Some useful subsystems can be:
 * DriveTrain - The movement of the bot
 * Lift - The lift mechanism of the bot
 * Intake/Outtake - The intake/outtake mechanisms of the bot
 *
 */
public abstract class Subsystem {
    protected final HardwareMap hardwareMap;
    protected final Telemetry telemetry;
    protected final boolean isAuton;

    /**
     * Creates a new instance of a subsystem
     *
     * @param  hardwareMap The hardware map used for the bot
     * @param telemetry The telemetry communicating between the bot and the drive controller
     * @param isAuton Whether or not this subsystem is being used from auton
     *
     */
    public Subsystem(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuton) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.isAuton = isAuton;
    }

    //region Loops
    /**
     * Update auton subsystem actions. Used to perform actions based on the current state.
     * Called every tick.
     *
     */
    public abstract void updateState();

    /**
     * Update teleOp the subsystem state. Used to perform actions based on gamepad inputs
     * Called every tick.
     *
     * @param GP1 The first gamepad to use (driving gamepad)
     * @param GP2 The second gamepad to use (other gamepad)
     */
    public abstract void updateTeleOpState(GamePadEx GP1, GamePadEx GP2);
    //endregion

    //region Init Loops
    /**
     * This loop is run only on init during TeleOp.
     * Called every tick on init.
     */
    public void initLoopTeleOp() {
    }

    /**
     * This loop is run only on init during Auton.
     * Called every tick on init.
     */
    public void initLoopAuton() {
    }
    //endregion

    //region Start Functions
    /**
     * This is called once on startup during TeleOp.
     */
    public void startTeleOp() {
    }

    /**
     * This is called once on startup during Auton.
     */
    public void startAuton() {
    }
    //endregion

    /**
     * This is called on stop.
     */
    public abstract void stop();
}