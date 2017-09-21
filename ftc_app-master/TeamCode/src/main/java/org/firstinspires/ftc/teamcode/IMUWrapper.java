package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 3256 on 9/20/2017.
 */

public class IMUWrapper {

    private String name;
    private HardwareMap hardwareMap;
    private BNO055IMU imu;

    public IMUWrapper(String name, HardwareMap hardwareMap){
        this.name = name;
        this.hardwareMap = hardwareMap;
    }

    public void initialize() {
        imu = hardwareMap.get(BNO055IMU.class, name);
    }

    public void calibrate() {

    }

    public void reset() {}

    public double getHeading() {
        return 0;
    }
}
