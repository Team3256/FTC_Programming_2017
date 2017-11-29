package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Team 3256 on 11/28/2017.
 */

public class Glyph{

    //DcMotor glyphIntake, glyphLinear;
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
        //glyphLinear = hardwareMap.dcMotor.get("glyphLinear");

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

    /*public String displayClampL(){
        return Double.toString(glyphClampL.getPosition());
    }

    public String displayClampR(){
        return Double.toString(glyphClampR.getPosition());
    } */

}
