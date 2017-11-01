package org.firstinspires.ftc.teamcode.base;

/**
 * Created by Team 3256 on 10/11/2017.
 */

public class PIDController {

    double P, I, D, kP, kI, kD, error, prevError, changeError, sumError, PID;


    public PIDController(double kP, double kI, double kD){

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

    }

    public void resetPID(){

        error = 0;
        prevError = 0;
        changeError = 0;
        sumError = 0;

    }

    public double getError (double current, double target){

        return Math.abs(current - target);

    }

    public double calculatePID(double current, double target){

        error = current - target;
        sumError += error;
        changeError = error - prevError;

        P = error;
        I = sumError;
        D = changeError;

        PID = (P*kP) + (I*kI) + (D*kD);

        prevError = error;

        return PID;


    }

}
