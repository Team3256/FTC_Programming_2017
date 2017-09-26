package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.sun.tools.doclint.HtmlTag.I;

/**
 * Created by Team 3256 on 9/20/2017.
 */
@TeleOp
public class Test extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DistanceSensor sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        IMUWrapper imu = new IMUWrapper("imu", hardwareMap);
        imu.calibrate();


        super.waitForStart();

        while (opModeIsActive()) {





            telemetry.addData("Distance (cm)", sensorColorRange.getDistance(DistanceUnit.CM));
            telemetry.addData("Color: Red", colorSensor.red());
            telemetry.addData("Color: Green", colorSensor.green());
            telemetry.addData("Color: Blue", colorSensor.blue());
            telemetry.update();
            telemetry.addData("Gyro:", imu.getHeading());
            telemetry.update();

            if (gamepad1.left_bumper){

                imu.reset();

            }


        }

    }

}