import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

class CalibrateGyro {

    final static String FILENAME = "BNO055IMUCalibration.json";

    private boolean readFromFile;
    private boolean writeComplete;

    /**
     * Construct a new CalibrateGyro object
     *
     * @param readFromFile true = read calibration data (teleop),
     *                     false = generate calibration data (auton)
     */
    public CalibrateGyro(boolean readFromFile) {
        this.readFromFile = readFromFile;
        this.writeComplete = false;
    }

    /**
     * Call this in init() as @code BNO055IMU imu = CalibrateGyro.initGyro(....);
     * NOTE: Above is why there was null pointer problem in tuesday's code --- MAKE SURE TO SET EQUAL ^^^
     *
     * @param hwMap      OpMode HardwareMap instance
     * @param configName OPTIONAL: name for configuration, by default "imu"
     * @return An instance of your gyro, fully initialized (but not necessarily done calibrating!)
     */
    public BNO055IMU initGyro(HardwareMap hwMap, String configName) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Enable logging
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        // If read calibration data from file, set param to filename
        if (this.readFromFile) {
            //File file = AppUtil.getInstance().getSettingsFile(FILENAME);
            //parameters.calibrationData = BNO055IMU.CalibrationData.deserialize(ReadWriteFile.readFile(file));
            parameters.calibrationDataFile = FILENAME;
        }

        // Initialize gyro with either data from file or new data
        BNO055IMU gyro = hwMap.get(BNO055IMU.class, configName);

        if (!this.readFromFile) {
            gyro.initialize(parameters);
        }

        return gyro;
    }

    // Helper so u can do less work -- uses default name for imu in config
    public BNO055IMU initGyro(HardwareMap hwMap) {
        return initGyro(hwMap, "imu");
    }

    /**
     * CALL THIS IN initLoop() once per loop
     *
     * @param gyro      your gyro instance
     * @param telemetry OpMode telemetry instance
     */
    public void initLoopGyro(BNO055IMU gyro, Telemetry telemetry) {

        // If we don't want to read from file, the gyro is calibrated,
        // and we have not already written to a file...
        if (!this.readFromFile && gyro.isGyroCalibrated() && !this.writeComplete) {
            // get calibration data from imu
            BNO055IMU.CalibrationData calibrationData = gyro.readCalibrationData();

            // write it to a json file
            File file = AppUtil.getInstance().getSettingsFile(FILENAME);
            ReadWriteFile.writeFile(file, calibrationData.serialize());

            // state that writing is complete
            this.writeComplete = true;
        }

        // output things about the gyro in telemetry
        for (String s : getCalibrationInfo(gyro)) {
            telemetry.addLine(s);
        }

        // output if the file write is complete -- ONLY PRESS START ON AUTON ONCE THIS SHOWS UP
        if (this.writeComplete) {
            telemetry.addLine("saved to " + FILENAME);
        }

        // update telemetry
        //telemetry.update();
    }

    private static List<String> getCalibrationInfo(BNO055IMU gyro) {
        List<String> telemetryData = new ArrayList<>();
        telemetryData.add("Sensor: IMU Gyro -----------");
        telemetryData.add("Status: " + gyro.getSystemStatus().toShortString());
        telemetryData.add("Calib Status: " + gyro.getCalibrationStatus().toString());
        telemetryData.add("Gyro Calib? " + gyro.isGyroCalibrated());
        telemetryData.add("heading: " + gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

        return telemetryData;
    }
}