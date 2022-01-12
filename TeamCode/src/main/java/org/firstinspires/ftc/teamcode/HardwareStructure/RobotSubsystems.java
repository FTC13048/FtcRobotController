package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotSubsystems {
    private CargoDropper cargoFlipper;
    private DistanceSensor distSensorBack, distSensorRight, distSensorLeft;
    private DriveTrain driver;
    private DuckSpinner duckSpinner;
    private Intake intake;
    private LinearSlide linSlide;

    private final double BUFFER_AREA = 0.15;
    private Telemetry telemetry;

    private boolean automateSlide = false;
    private boolean servoReset = false;
    private boolean debounceServo = false;
    private int armPos = -1;

    public RobotSubsystems(HardwareMap hmap, Telemetry tele, boolean isAuton){
        telemetry = tele;

        driver = new DriveTrain(hmap, tele, isAuton);
        cargoFlipper = new CargoDropper(hmap, tele);
        duckSpinner = new DuckSpinner(hmap, tele);
        intake = new Intake(hmap, tele);
        linSlide = new LinearSlide(hmap, tele);

        if(isAuton){
            distSensorBack = new DistanceSensor(hmap, tele, DistanceSensor.SensorName.BACK);
            distSensorLeft = new DistanceSensor(hmap, tele, DistanceSensor.SensorName.LEFT);
            distSensorRight = new DistanceSensor(hmap, tele, DistanceSensor.SensorName.RIGHT);
        }

        tele.addData("All subsystems", "Initialized");
    }

    // --------------------------------- TELE-OP METHODS ---------------------------------

    // runs the drive train with imput from the gamepads
    public void runDriveTrainTele(double leftStickY, double rightStickY,
                                  double rightTrigger, double leftTrigger){
        if(rightTrigger > 0.0){ driver.strafe(rightTrigger); }

        else if(leftTrigger > 0.0){ driver.strafe(-leftTrigger); }

        else{ driver.drive(leftStickY, rightStickY); }
    }

    // runs the intake given the trigger power from the gamepads
    public void runIntakeTele(double leftTrigger, double rightTrigger){
        if(leftTrigger > BUFFER_AREA){ intake.runIntake(); }

        else if(rightTrigger > BUFFER_AREA){ intake.runOuttake(); }

        else{ intake.setIntakePower(0.0); }
    }

    // runs the duck spinner given the stick power from the gamepads
    public void runDuckTele(double leftStickY){
        if(Math.abs(leftStickY) > BUFFER_AREA){ duckSpinner.setDuckPower(leftStickY * 0.7); }

        else{ duckSpinner.setDuckPower(0.0); }
    }

    public void runSlideTele(double rightStickY, boolean gamepadA){
        if(gamepadA){
            automateSlide = true;
        }

        if(automateSlide){
            if(linSlide.setLinSlidePos(-500)){
                cargoFlipper.setPos(0.3);
                automateSlide = false;
            }
        } else{
            if(Math.abs(rightStickY) > 0.0){ linSlide.runLinSlide(rightStickY * 0.5); }
            else{ linSlide.runLinSlide(0.0); }
            telemetry.addData("Slide position", linSlide.getLinSlidePos());
        }
    }

    // SERVO METHODS
    private void resetServo(boolean gamepadY){
        if(gamepadY){
            servoReset = true;
        }

        if(servoReset){
            cargoFlipper.setPos(0.1);
            servoReset = false;
        }
    }


    private void updateServo(boolean gamepadX){
        if(gamepadX){
            // Arm goes to pushing position if it is at rest
            if(!debounceServo && armPos == -1){
                cargoFlipper.setPos(0.9);
                armPos *= -1;
            }

            // Arm goes to rest position if a is pressed an it is pushing
            else if(!debounceServo && armPos == 1){
                cargoFlipper.setPos(0.1);
                armPos *= -1;
            }

            debounceServo = true;
        }

        // If a is not pressed, make sure the debounce variable is false
        else{
            debounceServo = false;
        }
    }

    // --------------------------- OTHER COMBO/NEEDED METHODS ---------------------------
    public int autonDrive(DirectionEnum movement, int target){
        return driver.autonDrive(movement, target);
    }

    public boolean turnAdjust(int degrees, double power){
        return driver.adjustHeading(degrees, power);
    }

    public boolean driveToDistance(double stopDist, double power, DirectionEnum movement,
                                   DistanceSensor.SensorName sensor){
        switch(sensor){
            case BACK:
                return driver.driveDistanceSensor(stopDist, power, distSensorBack, movement);

            case LEFT:
                return driver.driveDistanceSensor(stopDist, power, distSensorLeft, movement);

            case RIGHT:
                return driver.driveDistanceSensor(stopDist, power, distSensorRight, movement);
        }

        return false;
    }
}
