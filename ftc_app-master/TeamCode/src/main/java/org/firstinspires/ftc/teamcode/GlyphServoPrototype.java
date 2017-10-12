package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.Position;

import static android.R.attr.left;
import static android.R.attr.positiveButtonText;
import static android.R.attr.right;
import static com.sun.tools.doclint.HtmlTag.P;
import static com.sun.tools.javac.main.Option.D;

/**
 * Created by Team 3256 on 10/11/2017.
 */

@TeleOp

public class GlyphServoPrototype extends OpMode {

    Servo glyphServo1;
    Servo glyphServo2;
    //DcMotor linearMotion;

    double position = 0;

    @Override
    public void init() {
        glyphServo1 = hardwareMap.servo.get("glyphServo1");
        glyphServo2 = hardwareMap.servo.get("glyphServo2");
        //linearMotion = hardwareMap.dcMotor.get("glyphLift");

        glyphServo1.setPosition(glyphServo1.getPosition());
        glyphServo2.setPosition(glyphServo2.getPosition());

        //linearMotion.setDirection(DcMotorSimple.Direction.FORWARD);
        //linearMotion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //linearMotion.setPower(0);

    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper) {

            position += .001;

        }
        if (gamepad1.right_bumper) {

            position -= .001;

        }

        if (0 <= position && position <= 1){

            glyphServo1.setPosition(position);
            glyphServo2.setPosition(-1*position);
            telemetry.addData("Position", position);
        }
    }
}
