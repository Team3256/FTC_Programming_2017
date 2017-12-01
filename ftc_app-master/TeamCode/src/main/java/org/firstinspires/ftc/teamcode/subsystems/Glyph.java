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

        //glyphIntake = hardwareMap.dcMotor.get("glyphIntake");
        glyphElevator = hardwareMap.dcMotor.get("glyphElevator");
        glyphElevator.setDirection(DcMotorSimple.Direction.FORWARD);
        glyphElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //glyphIntakeL = hardwareMap.dcMotor.get("glyphIntakeL");
        //glyphIntakeR = hardwareMap.dcMotor.get("glyphIntakeR");

        glyphClampL = hardwareMap.servo.get("glyphClampL");
        glyphClampR = hardwareMap.servo.get("glyphClampR");
        //glyphPivotL = hardwareMap.servo.get("glyphPivotL");
        //glyphPivotR = hardwareMap.servo.get("glyphPivotR");

        glyphClampL.setPosition(glyphClampL.getPosition());
        glyphClampR.setPosition(glyphClampR.getPosition());
        //glyphPivotL.setPosition(glyphClampL.getPosition());
        //glyphPivotR.setPosition(glyphClampR.getPosition());
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
        glyphClampL.setPosition(1);
        glyphClampR.setPosition(0);
    }
    /*

    public void intake (){
        glyphIntakeL.setPower(0.5);
        glyphIntakeR.setPower(-0.5);
        //glyphPivotL.setPosition();
        //glyphPivotR.setPosition();
    }
    public void outtake (){
        glyphIntakeL.setPower(-0.5);
        glyphIntakeR.setPower(0.5);
        //glyphPivotL.setPosition();
        //glyphPivotR.setPosition();
    }
    public void zeroPower (){
        //glyphPivotL.setPosition();
        //glyphPivotR.setPosition();
        glyphIntakeL.setPower(0);
        glyphIntakeR.setPower(0);
    }*/

    public void resetEncoders() {

        glyphElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //public void manualElevator(double power){
        //glyphElevator.setPower(power);
    //}

    public double getElevatorEncoder(){

        return glyphElevator.getCurrentPosition();
    }

    public void elevatorFullUp(LinearOpMode opMode) {
        telemetryPass.addData("fullup","running");
        telemetryPass.update();
        /*while(glyphElevator.getCurrentPosition() < 1496) {
            glyphElevator.setPower(.5);
        }
        glyphElevator.setPower(0);*/
    }
    public void elevatorHalfUp(LinearOpMode opMode) {
        telemetryPass.addData("halfup","running");
        telemetryPass.update();/*
        double direction = -.3;
        if(glyphElevator.getCurrentPosition() < 1546/2) {
            direction = .5;
        }
        while(Math.abs(glyphElevator.getCurrentPosition()-1546/2) > 0) {
            glyphElevator.setPower(direction);
        }
        glyphElevator.setPower(0);*/
    }
    public void elevatorDown(LinearOpMode opMode) {

        telemetryPass.addData("down","running");
        telemetryPass.update();
        /*while(glyphElevator.getCurrentPosition() > 50) {
            glyphElevator.setPower(-.3);
        }
        glyphElevator.setPower(0);*/
    }

    /*public void elevatorFullUp() {
        while(glyphElevator.getCurrentPosition() < 700) {
            glyphElevator.setPower(0.8);
        }
        glyphElevator.setPower(0);
    }

    public void justUp(){
        glyphElevator.setPower(1);
    }

    public void elevatorHalfUp() {
        double direction = -.3;
        if(glyphElevator.getCurrentPosition() < 1700/2) {
            direction = .5;
        }
        while(Math.abs(glyphElevator.getCurrentPosition()-1700/2) > 0) {
            glyphElevator.setPower(direction);
        }
        glyphElevator.setPower(0);
    }

    public void elevatorDown() {
        while(glyphElevator.getCurrentPosition() > 50) {
            glyphElevator.setPower(-.3);
        }
        glyphElevator.setPower(0);
    } */

    /*public void elevatorFullUp(LinearOpMode opMode) {
        while(getElevatorEncoder() < 1496) {
            glyphElevator.setPower(.5);
        }
        glyphElevator.setPower(0);
    }

    public void elevatorHalfUp(LinearOpMode opMode) {
        double direction = (getElevatorEncoder() > 773) ? -.3 : .5;
        while(Math.abs(getElevatorEncoder()-773) > 0) {
            glyphElevator.setPower(direction);
        }
        glyphElevator.setPower(0);
    }

    public void elevatorDown(LinearOpMode opMode) {
        while(getElevatorEncoder() > 50) {
            glyphElevator.setPower(-.3);
        }
        glyphElevator.setPower(0);
    } */

    //1546
    /*public String displayClampL(){
        return Double.toString(glyphClampL.getPosition());
    }

    public String displayClampR(){
        return Double.toString(glyphClampR.getPosition());
    } */

}
