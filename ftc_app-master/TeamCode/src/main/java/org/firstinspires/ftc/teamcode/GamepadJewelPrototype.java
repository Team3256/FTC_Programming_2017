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
public class GamepadJewelPrototype extends OpMode{

    Servo jewelArm;

    @Override

    public void init() {

        jewelArm = hardwareMap.servo.get("jewel_arm");
    }

    @Override
    public void loop() {

        if (gamepad1.a) {

            jewelArm.setPosition(0.2);

        }
        else if (gamepad1.y) {

            jewelArm.setPosition(-0.2);

        }
        else {

            jewelArm.setPosition(0);
        }
    }
}