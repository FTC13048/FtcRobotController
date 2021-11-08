package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;

// This class is mostly for debugging. When called, it will display an overview of the bot, showing various stats from motor directions and speeds to sensor data
public class BotView {
    // Declare bot stuff
    private Telemetry telemetry;
    private HardwareMap map;

    // Declare the selected display mode
    public DisplayMode CurrentMode;

    // Declare the selectable display modes
    public enum DisplayMode {
        OVERVIEW,
        MOTOR,
        SENSOR,
    }

    // Initialize values and references
    public BotView(HardwareMap hmap, Telemetry tele) {
        this.map = hmap;
        telemetry = tele;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setCaptionValueSeparator("");
        telemetry.setItemSeparator("");
    }

    // Shows an general overview of the bots data
    /*
    Displays:
    - A diagram of the motors indicating for each motor's speeds (wheel, slide, spinner, and intake)
    - A diagram of the bots direction (using gyro)
    */
    public void ShowBotView(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, DcMotor slide, DcMotor intakeL, DcMotor intakeR, DcMotor spinner, double botDir) {
        // Get motor power
        double flPower = fl.getPower();
        double frPower = fr.getPower();
        double blPower = bl.getPower();
        double brPower = bl.getPower();
        double slidePower = slide.getPower();
        double spinnerPower = spinner.getPower();
        double intakePowerL = intakeL.getPower();
        double intakePowerR = intakeR.getPower();

        // Set the motor power indicators
        String flInd, frInd, blInd, brInd;
        String slideInd, spinnerInd, intakeIndL, intakeIndR;
        flInd = flPower == 0 ? "[00]" : flPower > 0 ? "//\\" : flPower < 0 ? "\\//" : "";
        frInd = frPower == 0 ? "[00]" : frPower > 0 ? "//\\" : frPower < 0 ? "\\//" : "";
        blInd = blPower == 0 ? "[00]" : blPower > 0 ? "//\\" : blPower < 0 ? "\\//" : "";
        brInd = brPower == 0 ? "[00]" : brPower > 0 ? "//\\" : brPower < 0 ? "\\//" : "";
        slideInd = slidePower == 0 ? "[00]" : slidePower > 0 ? "//\\" : slidePower < 0 ? "\\//" : "";
        spinnerInd = spinnerPower == 0 ? "[00]" : spinnerPower > 0 ? "//\\" : spinnerPower < 0 ? "\\//" : "";
        intakeIndL = intakePowerL == 0 ? "[00]" : intakePowerL > 0 ? "//\\" : intakePowerL < 0 ? "\\//" : "";
        intakeIndR = intakePowerR == 0 ? "[00]" : intakePowerR > 0 ? "//\\" : intakePowerR < 0 ? "\\//" : "";

        // Draw the motor stats diagram
        telemetry.addLine();
        telemetry.addData("Motor Stats:", "");
        telemetry.addData("FL     " + flInd + "  ", "  " + frInd + "     FR");
        telemetry.addLine();
        telemetry.addData("BL     " + blInd + "  ", "  " + brInd + "     BR");
        telemetry.addLine();
        telemetry.addData("Slide  " + slideInd + "  ", "  " + spinnerInd + " Spinner");
        telemetry.addLine();
        telemetry.addData("IntakeL  " + intakeIndL + "  ", "  " + intakeIndR + " IntakeR");

        // Draw the motor direction as a bar thingy
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Bot Direction:", "");
        String headingBar = "";

        for (int i = 0; i < 10; i++) {
            if (((int) botDir / 10) == i)
                headingBar.concat(" <->");
            else
                headingBar.concat(" -");
        }

        telemetry.addData("|" + headingBar + " |", "");
    }

    // Shows a detailed view of the bots motor data
    /*
    Displays:
    - For each wheel motor
        * Direction
        * Set power
        * Position (from encoder)
    - Liner slide
        * Extension amount (from encoder)
        * Set power
        * Position (from encoder)
    - Duck spinner
        * Set power
        * Position (from encoder)
    - Basket servo
        * Actual position (from encoder)
    - For each intake motor
        * Set power
        * Position (from encoder)
    - The bots current action (turn, drive, stop)
    - The bots overall velocity
    */
    public void ShowMotorView(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, DcMotor slide, DcMotor intakeL, DcMotor intakeR, DcMotor spinner, Servo basket, MovementEnum state) {
        // Get wheel motors powers
        double flPower = fl.getPower();
        double frPower = fr.getPower();
        double blPower = bl.getPower();
        double brPower = br.getPower();

        // Get intake motors powers
        double intakeLPower = intakeL.getPower();
        double intakeRPower = intakeR.getPower();

        // Get slide stats
        double slideMotorPower = slide.getPower();
        double slidePosition = slide.getCurrentPosition();

        // Get spinner power
        double spinnerPower = spinner.getPower();

        // Get basket servo stats
        double basketServoTarget = 0;
        Servo.Direction basketServoDir = Servo.Direction.FORWARD;
        if (basket != null) {
            basketServoTarget = basket.getPosition();
            basketServoDir = basket.getDirection();
        }

        // Display wheel motor powers
        telemetry.addData("Front Left Motor Power: ", flPower);
        telemetry.addData("Front Right Motor Power: ", frPower);
        telemetry.addData("Back Left Motor Power: ", blPower);
        telemetry.addData("Back Right Motor Power: ", brPower);
        telemetry.addLine();

        // Display slide stats (position and motor power)
        telemetry.addData("Slide Extension: ", slidePosition);
        telemetry.addData("Slide Motor Power: ", slideMotorPower);
        telemetry.addLine();

        // Display spinner motor power
        telemetry.addData("Spinner Motor Power: ", spinnerPower);
        telemetry.addLine();

        // Display intake motors powers
        telemetry.addData("Left Intake Motor Power: ", intakeLPower);
        telemetry.addData("Right Intake Motor Power: ", intakeRPower);
        telemetry.addLine();

        // Display basket servo stats
        telemetry.addData("Basket Servo Position: ", basketServoTarget);
        telemetry.addData("Basket Servo Direction: ", basketServoDir);
        telemetry.addLine();

        // Display the current movement state
        telemetry.addData("Current Movement State: ", state);
    }

    // Shows a detailed view of the bots sensor data
    /*
    Displays:
    - The bots direction (from gyro)
    - Distance sensor
        * Sound detected distance
        * Light amount detected
    - Any other sensor data too (I just need to add its data to the code)
    */
    public void ShowSensorView(double botDir, double soundDist, double lightAmount) {
        telemetry.addData("Gyro Sensor:", "");
        String headingBar = "";

        for (int i = 0; i < 10; i++) {
            if ((botDir / 10) == i)
                headingBar.concat(" <->");
            else
                headingBar.concat(" -");
        }

        telemetry.addData("|" + headingBar + " |", ": " + botDir);
        telemetry.addLine();

        telemetry.addData("Ultrasonic Sensor Distance: ", soundDist);
        telemetry.addData("Light Sensot Amount: ", lightAmount);
    }
}