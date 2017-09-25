package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Team 3256 on 9/22/2017.
 */
@TeleOp (name = "Gamepad Jewel", group = "Test")
public class GamepadJewelPrototype extends OpMode {

    Servo jewelArm;
    double position = 0;

    @Override

    public void init() {

        jewelArm = hardwareMap.servo.get("jewel_arm");
        jewelArm.setPosition(jewelArm.getPosition());
    }

    @Override
    public void loop() {

        if (gamepad1.y) {

            position -= 0.001;

        } else if (gamepad1.a) {

            position += 0.001;

        }

        if (position > 1) {

            position = 1;

        }

        else if (position < 0) {

            position = 0;

        }

        jewelArm.setPosition(position);
    }

}