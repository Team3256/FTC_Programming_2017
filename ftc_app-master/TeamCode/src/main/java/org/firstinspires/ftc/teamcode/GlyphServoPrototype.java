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
import static android.R.attr.paddingStart;
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
    DcMotor linearMotion;
    double power;

    double position = 0;

    @Override
    public void init() {
        glyphServo1 = hardwareMap.servo.get("glyphServo1");
        glyphServo2 = hardwareMap.servo.get("glyphServo2");
        linearMotion = hardwareMap.dcMotor.get("glyphLift");

        glyphServo1.setPosition(glyphServo1.getPosition());
        glyphServo2.setPosition(glyphServo2.getPosition());

        linearMotion.setDirection(DcMotorSimple.Direction.FORWARD);
        linearMotion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearMotion.setPower(power);

        telemetry.addData(Double.toString(glyphServo1.getPosition()), Double.toString(glyphServo1.getPosition() + 0.5));

        telemetry.addData(Double.toString(glyphServo2.getPosition() - 0.5), Double.toString(glyphServo2.getPosition()));

    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper) {

            position += .007;

        }
        if (gamepad1.right_bumper) {

            position -= .007;

        }

        if (gamepad1.y) {
            power = .007;
        }

        else {
            power = 0;
        }

        position = Range.clip(position, 0, 1);
        power = Range.clip(power, -1, 1);

        glyphServo1.setPosition(position);
        glyphServo2.setPosition(1 - position);
        linearMotion.setPower(power);
        telemetry.addData("Position", position);
        telemetry.update();

    }
}
