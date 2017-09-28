package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static android.R.attr.left;

/**
 * Created by Team 6696 on 9/25/2017.
 */

public class DriveTrain {

    DcMotor leftFront, rightFront;

    private static DriveTrain driveTrain = new DriveTrain();

    private DriveTrain(){

    }

    public void init(HardwareMap hardwareMap){

        //Initializing motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        //leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        //rightBack = hardwareMap.dcMotor.get("rightBack");

        //set motor directions

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        //leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //set motors to brake when inactive

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setPower(0);

    }

    public void setPower(double power){
        runLeft(power);
        runRight(power);
    }

    public void runLeft(double speed){
        leftFront.setPower(speed);
        //leftBack.setPower(speed);
    }
    public void runRight(double speed){
        rightFront.setPower(speed);
        //rightBack.setPower(speed);
    }

    public void tankDrive(double left, double right){
        runLeft(left);
        runRight(right);
    }

    public void setMode (DcMotor.RunMode mode){
        //leftBack.setMode(mode);
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        //rightBack.setMode(mode);
    }

    public static DriveTrain getInstance() {
        return driveTrain;
    }


}