package org.firstinspires.ftc.teamcode.prototypes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Team 3256 on 10/18/2017.
 */

@TeleOp

public class RelicPrototype extends OpMode {

    Servo relicServo;
    double position = 0;

    @Override
    public void init() {

        relicServo = hardwareMap.servo.get("relicServo");
        relicServo.setPosition(relicServo.getPosition());

    }

    @Override
    public void loop() {

        if (gamepad1.x){
            position += .007;
        }

        else if (gamepad1.b){
            position -= .007;
        }

        position = Range.clip(position, 0, 0.5);
        relicServo.setPosition(position);

    }
}
