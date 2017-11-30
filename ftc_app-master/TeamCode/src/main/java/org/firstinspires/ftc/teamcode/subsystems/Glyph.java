package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Team 3256 on 11/28/2017.
 */

public class Glyph{

    DcMotor glyphElevator; //need to add glyphIntake
    Servo glyphClampL, glyphClampR;
    private HardwareMap hardwareMap;

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

        glyphClampL = hardwareMap.servo.get("glyphClampL");
        glyphClampR = hardwareMap.servo.get("glyphClampR");

        glyphClampL.setPosition(glyphClampL.getPosition());
        glyphClampR.setPosition(glyphClampR.getPosition());
    }

    public void clampIn(){
        glyphClampL.setPosition(1);
        glyphClampR.setPosition(0);
    }

    public void clampOut(){
        glyphClampL.setPosition(0);
        glyphClampR.setPosition(1);
    }

    public void resetEncoders() {

        glyphElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getElevatorEncoder(){
        return glyphElevator.getCurrentPosition();
    }

    public void elevatorFullUp(LinearOpMode opMode) {
        while(glyphElevator.getCurrentPosition() < 1496) {
            glyphElevator.setPower(.5);
        }
        glyphElevator.setPower(0);
    }

    public void elevatorHalfUp(LinearOpMode opMode) {
        double direction = -.3;
        if(glyphElevator.getCurrentPosition() < 1546/2) {
            direction = .5;
        }
        while(Math.abs(glyphElevator.getCurrentPosition()-1546/2) > 0) {
            glyphElevator.setPower(direction);
        }
        glyphElevator.setPower(0);
    }

    public void elevatorDown(LinearOpMode opMode) {
        while(glyphElevator.getCurrentPosition() > 50) {
            glyphElevator.setPower(-.3);
        }
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
