package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.DriveTrainTeleop.telemetryPass;

/**
 * Created by Team 3256 on 11/28/2017.
 */

public class Glyph{

    DcMotor glyphElevator, glyphIntakeL, glyphIntakeR; //need to add glyphIntake
    Servo glyphClampL, glyphClampR, glyphPivotL, glyphPivotR;
    private HardwareMap hardwareMap;
    boolean in = true;

    private static Glyph glyph = new Glyph();

    private Glyph(){

    }

    public static Glyph getInstance(){
        return glyph;
    }

    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        //Initializing motors and servos

        glyphElevator = hardwareMap.dcMotor.get("glyphElevator");
        glyphElevator.setDirection(DcMotorSimple.Direction.FORWARD);
        glyphElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphIntakeL = hardwareMap.dcMotor.get("glyphIntakeL");
        glyphIntakeR = hardwareMap.dcMotor.get("glyphIntakeR");

        glyphClampL = hardwareMap.servo.get("glyphClampL");
        glyphClampR = hardwareMap.servo.get("glyphClampR");
        glyphPivotL = hardwareMap.servo.get("glyphPivotL");
        glyphPivotR = hardwareMap.servo.get("glyphPivotR");

        glyphClampL.setPosition(glyphClampL.getPosition());
        glyphClampR.setPosition(glyphClampR.getPosition());
        glyphPivotL.setPosition(glyphClampL.getPosition());
        glyphPivotR.setPosition(glyphClampR.getPosition());
    }

    public void clampToggle(){
        if (in){
            clampOut();
            in = false;
        }
        else {
            clampIn();
            in = true;
        }
    }

    public void clampIn(){
        glyphClampL.setPosition(0);
        glyphClampR.setPosition(1);
    }

    public void clampOut(){
        glyphClampL.setPosition(0.47);
        glyphClampR.setPosition(0.53);
    }


    public void intake (){
        glyphIntakeL.setPower(-0.5);
        glyphIntakeR.setPower(0.5);
        glyphPivotL.setPosition(0.9);
        glyphPivotR.setPosition(0.25);
    }
    public void outtake (){
        glyphIntakeL.setPower(0.5);
        glyphIntakeR.setPower(-0.5);
        glyphPivotL.setPosition(0.9);
        glyphPivotR.setPosition(0.25);
    }
    public void zeroPower (){
        glyphPivotL.setPosition(0.71);
        glyphPivotR.setPosition(0.45);
        glyphIntakeL.setPower(0);
        glyphIntakeR.setPower(0);
    }

    public void intakeOut(){
        glyphPivotR.setPosition(1);
        glyphPivotL.setPosition(0);
        glyphIntakeR.setPower(0);
        glyphIntakeL.setPower(0);
    }

    public void resetEncoders() {

        glyphElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //public void manualElevator(double power){
        //glyphElevator.setPower(power);
    //}

    public double getElevatorEncoder(){

        return glyphElevator.getCurrentPosition();
    }

    public void goUp() {
        glyphElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        glyphElevator.setPower(-.7);

    }

    public void goDown() {
        glyphElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        glyphElevator.setPower(.7);

    }

    public void elevatorFullUp( ) {
        glyphElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        glyphElevator.setTargetPosition(-10);
        glyphElevator.setPower(-0.2);
        /*
        telemetryPass.addData("fullup","running");
        telemetryPass.update();
        if(glyphElevator.getCurrentPosition() > -1496) {
            glyphElevator.setPower(.5);
        }

        else {
            glyph.stopElevator(opMode);
        }
        */
    }
    public void elevatorHalfUp() {
        glyphElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glyphElevator.setTargetPosition(-1496/2);
        glyphElevator.setPower(-0.2);
        /*telemetryPass.addData("halfup","running");
        telemetryPass.update();
        double direction = -.3;
        if(glyphElevator.getCurrentPosition() > -1546/2) {
            direction = .5;
        }

        else {
            glyph.stopElevator(opMode);
        }

        if(Math.abs(glyphElevator.getCurrentPosition()+1546/2) > 0) {
            glyphElevator.setPower(direction);
        }

        else {
            glyph.stopElevator(opMode);
        }
        */
    }
    public void elevatorDown( ) {
        glyphElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glyphElevator.setTargetPosition(-1496);
        glyphElevator.setPower(-0.2);

        /*
        telemetryPass.addData("down","running");
        telemetryPass.update();
        if(glyphElevator.getCurrentPosition() <  -50) {
            glyphElevator.setPower(-.3);
        }
        else {
            glyph.stopElevator(opMode);
        }*/
    }

    public void stopElevator( ) {
        glyphElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glyphElevator.setPower(0);

    }



    //1546
    /*public String displayClampL(){
        return Double.toString(glyphClampL.getPosition());
    }

    public String displayClampR(){
        return Double.toString(glyphClampR.getPosition());
    } */

}
