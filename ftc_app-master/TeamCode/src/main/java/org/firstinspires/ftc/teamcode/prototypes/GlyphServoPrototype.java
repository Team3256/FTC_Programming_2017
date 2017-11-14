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

    Servo glyphClaw1;
    Servo glyphClaw2;
    Servo rightPivot;
    Servo leftPivot;
    DcMotor linearMotion;
    DcMotor intakeMotor1;
    DcMotor intakeMotor2;
    double power;
    double intakePower = 0;

    double position = 0;
    double pivotPos1 = 1;
    double pivotPos2 = 0;

    int counter = 0;

    @Override
    public void init() {
        /*
        glyphClaw1 = hardwareMap.servo.get("glyphClaw1");
        glyphClaw2 = hardwareMap.servo.get("glyphClaw2");*/

        rightPivot = hardwareMap.servo.get("glyphPivot1");
        leftPivot = hardwareMap.servo.get("glyphPivot2");
        /*
        linearMotion = hardwareMap.dcMotor.get("glyphLift");
        intakeMotor1 = hardwareMap.dcMotor.get("intakeMotor1");
        intakeMotor2 = hardwareMap.dcMotor.get("intakeMotor2");



        glyphClaw1.setPosition(glyphClaw1.getPosition());
        glyphClaw2.setPosition(glyphClaw2.getPosition());
        */


        rightPivot.setPosition(rightPivot.getPosition());
        rightPivot.setDirection(Servo.Direction.REVERSE);
        leftPivot.setPosition(leftPivot.getPosition());
        leftPivot.setDirection(Servo.Direction.FORWARD);

        /*
        linearMotion.setDirection(DcMotorSimple.Direction.FORWARD);
        linearMotion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */

        //linearMotion.setPower(power);
        //intakeMotor1.setPower(intakePower);
        //intakeMotor2.setPower(intakePower);
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
            rightPivot.setPosition(1); //0.02128
            leftPivot.setPosition(1);
            //rightPivot.setPosition(1);
            telemetry.addData("pressed","an");
            //intakePower = 1;
        } else {
            rightPivot.setPosition(0);
            leftPivot.setPosition(0);
            //rightPivot.setPosition(0);
            //intakePower = 0;
        }




        position = Range.clip(position, 0, 1);
        power = Range.clip(power, -1, 1);

        //intakeMotor1.setPower(-intakePower);
        //intakeMotor2.setPower(intakePower);

        /*
        glyphClaw1.setPosition(position);
        glyphClaw2.setPosition(1 - position);*/
        //linearMotion.setPower(power);
        telemetry.addData("getPosition()right returns", pivotPos1);
        telemetry.addData("getPosition()left returns", pivotPos2);
        telemetry.update();

    }
}
