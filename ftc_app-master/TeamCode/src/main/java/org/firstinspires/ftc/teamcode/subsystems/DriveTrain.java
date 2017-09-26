package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 6696 on 9/25/2017.
 */

public class DriveTrain {

    DcMotor leftFront, leftBack, rightFront, rightBack;

    public void init(HardwareMap hardwareMap){
        //Initializing motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
    }

    public void runLeft(double speed){
        leftFront.setPower(speed);
        leftBack.setPower(speed);
    }
    public void runRight(double speed){
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }

    public void tankDrive(double left, double right){
        runLeft(left);
        runRight(right);
    }


}
