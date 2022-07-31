package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A boilerplate subsystem that you can use to quickly and easily create new subsystems
 *
 * Each subsystem on the robot (i.e. the intake and the drivetrain) will have their own
 * subsystem. This makes the code more modular so code can quickly be adapted if robot
 * parts and systems are changed. It also makes the code cleaner in general.
 */
public class BoilerplateSubsystem extends Subsystem {
    // declare all hardware and any variables needed to change hardware speed or position
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
    // This is where the system sends commands to the actual hardware based on the state

    // ALL HARDWARE SHOULD BE MODIFIED IN THIS METHOD
    public void updateState() {

    }

    @Override
    // This is where the system is told what inputs make it move states
    //     i.e. if the system is in STATE1 this is where it is told how it can
    //     move to STATE2

    // NO HARDWARE SHOULD BE DIRECTLY MODIFIED IN THIS METHOD
    public void updateTeleOpState(GamePadEx gp1, GamePadEx gp2) {

    }

    @Override
    public void stop() {

    }

    // Each subsystem will need a state enum
    public enum SubsystemState{
        STATE1, STATE2
    }
}