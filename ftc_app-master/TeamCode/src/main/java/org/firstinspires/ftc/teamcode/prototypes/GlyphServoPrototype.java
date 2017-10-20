package org.firstinspires.ftc.teamcode.prototypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.Position;

import static android.R.attr.hardwareAccelerated;
import static android.R.attr.imeFullscreenBackground;
import static android.R.attr.left;
import static android.R.attr.paddingStart;
import static android.R.attr.positiveButtonText;
import static android.R.attr.right;
import static android.R.attr.searchHintIcon;
import static com.sun.tools.doclint.HtmlTag.P;
import static com.sun.tools.javac.main.Option.D;

/**
 * Created by Team 3256 on 10/11/2017.
 */

@TeleOp

public class GlyphServoPrototype extends OpMode {

    Servo glyphServo1;
    Servo glyphServo2;
    Servo glyphIntake1;
    DcMotor linearMotion;
    DcMotor intakeMotor1;
    double power;
    double intakePower = 0;
    //boolean intaked = false;
    double position = 0;

    @Override
    public void init() {
        glyphServo1 = hardwareMap.servo.get("glyphServo1");
        glyphServo2 = hardwareMap.servo.get("glyphServo2");
        glyphIntake1 = hardwareMap.servo.get("glyphIntake1");
        linearMotion = hardwareMap.dcMotor.get("glyphLift");
        intakeMotor1 = hardwareMap.dcMotor.get("intakeMotor1");

        glyphServo1.setPosition(glyphServo1.getPosition());
        glyphServo2.setPosition(glyphServo2.getPosition());
        glyphIntake1.setPosition(glyphIntake1.getPosition());
        glyphIntake1.setDirection(Servo.Direction.FORWARD);

        linearMotion.setDirection(DcMotorSimple.Direction.FORWARD);
        linearMotion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearMotion.setPower(power);
        intakeMotor1.setPower(intakePower);
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
            power = -.75;
        }


        else if (gamepad1.a){
            power = .5;
        }

        else{
            power = 0;
        }

        if (gamepad1.b) {
            glyphIntake1.setPosition(0.116);
            intakePower = .75;
        } else {
            glyphIntake1.setPosition(0.097);
            intakePower = 0;
        }

        intakeMotor1.setPower(intakePower);

        position = Range.clip(position, 0, 1);
        power = Range.clip(power, -1, 1);

        glyphServo1.setPosition(position);
        glyphServo2.setPosition(1 - position);
        linearMotion.setPower(power);
        telemetry.addData("Position", position);
        telemetry.update();

    }
}
