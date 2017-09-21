package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Team 3256 on 9/20/2017.
 */

public class IMUWrapper {

    private String name;
    private HardwareMap hardwareMap;
    private BNO055IMU imu;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public IMUWrapper(String name, HardwareMap hardwareMap){
        this.name = name;
        this.hardwareMap = hardwareMap;
    }

    public void calibrate() {

        imu = hardwareMap.get(BNO055IMU.class, name);
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
        String filename = "AdafruitIMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());

    }

    public AngularVelocity getHeading() {
        return imu.getAngularVelocity();
    }

    public void reset() {

    }

}
