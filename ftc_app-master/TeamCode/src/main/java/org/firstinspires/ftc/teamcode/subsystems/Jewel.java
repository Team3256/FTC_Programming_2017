package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Team 3256 on 12/1/2017.
 */

public class Jewel {
    Servo jewelArm;
    double position = 0;
    ColorSensor colorSensor;
    DistanceSensor sensorColorRange;

    private static Jewel jewel = new Jewel();

    public void init(HardwareMap hardwareMap) {

        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelArm.setPosition(jewelArm.getPosition());
        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
    }

    public static Jewel getInstance(){
        return jewel;
    }

    public void resetArm(){
        jewelArm.setPosition(jewelArm.getPosition());
    }

    public void setArmUp(){
        position = 0; //0.5
        jewelArm.setPosition(position);
    }

    public void setArmDown(){
        position = 0.82;
        jewelArm.setPosition(position);
    }

    public void jewelUp(){
        position += 0.001;
        position = Range.clip(position, 0, 1);
        jewelArm.setPosition(position);
    }

    public void jewelDown(){
        position -= -0.001;
        position = Range.clip(position, 0, 1);
        jewelArm.setPosition(position);
    }

    public boolean isBlue(){
        if (colorSensor.red() >= (2 * colorSensor.blue())) {
            return false;
        }
        return true;
    }
}
