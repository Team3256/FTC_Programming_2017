package org.firstinspires.ftc.teamcode.subsystems;

import android.hardware.camera2.CameraConstrainedHighSpeedCaptureSession;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.io.instructions.ThreeRegisterDecodedInstruction;
import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.Constant;
import org.firstinspires.ftc.teamcode.base.Constants;
import org.firstinspires.ftc.teamcode.base.PIDController;

import static android.R.attr.breakStrategy;
import static android.R.attr.left;
import static android.R.attr.right;
import static com.sun.tools.javac.jvm.ByteCodes.bool_and;
import static com.sun.tools.javac.jvm.ByteCodes.error;
import static org.firstinspires.ftc.teamcode.DriveTrainTeleop.telemetryPass;


/**
 * Created by Team 2891 on 9/25/2017.
 */

//Created by Team 2891

@TeleOp
public class DriveTrain {

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    private static DriveTrain driveTrain = new DriveTrain();

    private HardwareMap hardwareMap;

    private IMUWrapper gyro;

    private DriveTrain(){

    }

    public void init(HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;

        gyro = new IMUWrapper("imu", hardwareMap);

        telemetryPass.addData("hardwareMap",hardwareMap==null);
        telemetryPass.update();

        gyro.calibrate();

        //Initializing motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        //set motor directions

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //set motors to brake when inactive

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        //setPower(0);

    }

    public void setPower(double power){
        runLeft(power);
        runRight(power);
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

    public void setMode (DcMotor.RunMode mode){
        leftBack.setMode(mode);
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    //Flip Motor Direction
    public void flipDirection (){
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //Unflip Motor Direction
    public void unFlipDirection (){
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void arcadeDrive(double throttle, double turn){

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double left = Math.pow((throttle + turn), 1);
        double right = Math.pow((throttle - turn), 1);

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1)*.93;

        runLeft(left);
        runRight(right);

        telemetryPass.addData("Right Front", rightFront.getCurrentPosition());
        telemetryPass.addData("Right Back", rightBack.getCurrentPosition());
        telemetryPass.addData("Left Front", leftFront.getCurrentPosition());
        telemetryPass.addData("Left Back", leftBack.getCurrentPosition());
    }

    public static DriveTrain getInstance() {
        return driveTrain;
    }

    public double inchesToDegrees(double inches){
        return (inches*360)/(Constants.WHEEL_DIAMETER * Math.PI * Constants.GEAR_RATIO);
    }

    public double degreesToTicks(double degrees){
        return (degrees * Constants.TICKS_PER_ROTATION)/360;
    }

    public double inchesToTicks(double inches){
        return degreesToTicks(inchesToDegrees(inches));
    }

    public double ticksToDegrees(double ticks){
        return ticks*(360/Constants.TICKS_PER_ROTATION);
    }

    public double degreesToInches(double degrees){
        return (Constants.WHEEL_DIAMETER*Math.PI)*(degrees/360);
    }

    public double ticksToInches(double ticks) { return degreesToInches(ticksToDegrees(ticks)); }

    public void setTargetPos(int pos){
        leftFront.setTargetPosition(pos);
        leftBack.setTargetPosition(pos);
        rightFront.setTargetPosition(pos);
        rightBack.setTargetPosition(pos);
    }

    public double getRightEncoder(){
        return (Math.abs(rightFront.getCurrentPosition())+Math.abs(rightBack.getCurrentPosition()))/2;
    }

    public double getLeftEncoder(){
        return (Math.abs(leftFront.getCurrentPosition())+Math.abs(leftBack.getCurrentPosition()))/2;
    }

    public double getAverageEncoderValue(){
        return (getRightEncoder()+getLeftEncoder())/2;
    }

    public void resetEncoders(){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void driveToDistance(double inches, boolean forward, double timeout, LinearOpMode opMode) throws InterruptedException {
        if (!forward){flipDirection();}
        int ticks = (int)inchesToTicks(inches);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDController distancePIDController = new PIDController(0.001, 0.0 , 0.0006); //distance PID
        PIDController gyroPIDController = new PIDController(0.00000000025, 0, 0.00005);
        gyro.reset();
        resetEncoders();
        double startTime = System.currentTimeMillis();
        double errorDistance = 0;
        while(System.currentTimeMillis() - startTime <= timeout*1000 && opMode.opModeIsActive()){

            double distancePID = distancePIDController.calculatePID(getAverageEncoderValue(), ticks);

            double gyroPID = gyroPIDController.calculatePID(gyro.getHeading(), 0) * (forward ? 1 : -1);

            if (gyroPIDController.getError(gyro.getHeading(), 0) <= 1.75){gyroPID = 0;} //possibly comment out
            errorDistance = distancePIDController.getError(getAverageEncoderValue(), ticks);

            if (errorDistance <= 5) {
                break;
            }

            runRight(distancePID + gyroPID);
            runLeft(distancePID - gyroPID);

            telemetryPass.addData("Error",error);
            telemetryPass.addData("Distance PID: ", distancePID);
            telemetryPass.addData("Gyro", gyro.getHeading());
            telemetryPass.addData("Left encoder ticks",getLeftEncoder());
            telemetryPass.addData("Right encoder ticks",getRightEncoder());
            telemetryPass.addData("Inches",ticksToInches(getAverageEncoderValue()));
            telemetryPass.update();
        }

        runRight(0);
        runLeft(0);


        telemetryPass.addData("Right Front", rightFront.getCurrentPosition());
        telemetryPass.addData("Right Back", rightBack.getCurrentPosition());
        telemetryPass.addData("Left Front", leftFront.getCurrentPosition());
        telemetryPass.addData("Left Back", leftBack.getCurrentPosition());

        telemetryPass.addData("Loop","finished");
        telemetryPass.addData("Error",error);
        telemetryPass.addData("Gyro", gyro.getHeading());
        telemetryPass.addData("Left encoder ticks",getLeftEncoder());
        telemetryPass.addData("Right encoder ticks",getRightEncoder());
        telemetryPass.addData("Inches",degreesToInches(ticksToDegrees(getAverageEncoderValue())));
        telemetryPass.update();

        Thread.sleep(20000);

    }


    public void turnWithPID(int degrees, double timeout, double power, LinearOpMode opMode) throws InterruptedException {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetEncoders();
        gyro.reset();
        double startTime = System.currentTimeMillis();
        PIDController turnPIDController = new PIDController(0.01, 0, 0);
        double targetTicks = degreesToTicks(Constants.ROBOT_TRACK*degrees/(Constants.WHEEL_DIAMETER));
        double rightPower = power;
        double leftPower = power;
        boolean reachedTarget = false;
        boolean turnRight = false;

        if (degrees > 0){
            leftPower *= -1; //.93
        }

        else {
            rightPower *= -1;
        }

        while(System.currentTimeMillis() - startTime <= timeout*1000 && opMode.opModeIsActive() && reachedTarget == false){

            reachedTarget = ((Math.abs(getLeftEncoder()) > Math.abs(targetTicks)) && (Math.abs(getRightEncoder()) > Math.abs(targetTicks)));

            runRight(rightPower);
            runLeft(leftPower);

            telemetryPass.addData("Target Ticks", targetTicks);
            telemetryPass.addData("Left Encoder Ticks", getLeftEncoder());
            telemetryPass.addData("Right Encoder Ticks", getRightEncoder());
            telemetryPass.update();
        }
        runRight(0);
        runLeft(0);

        Thread.sleep(2000);

        if (degrees - gyro.getHeading() < 0){
            turnRight = true;
        }

        double turnPID;

        while(System.currentTimeMillis() - startTime <= timeout*1000 && opMode.opModeIsActive()){

            turnPID = turnPIDController.calculatePID(gyro.getHeading(), -degrees) * (turnRight ? 1 : -1);

            if ((Math.abs(gyro.getHeading() + degrees) <= 0.75)){
                break;
            }
//Albert was here on 11/28/2017
            runRight(turnPID);
            runLeft(-turnPID);
            telemetryPass.addData("Target Ticks", targetTicks);
            telemetryPass.addData("Left Encoder Ticks", getLeftEncoder());
            telemetryPass.addData("Right Encoder Ticks", getRightEncoder());
            telemetryPass.addData("Gyro", gyro.getHeading());
            telemetryPass.addData("TurnPID", turnPID);
            telemetryPass.addData("Target", degrees);
            telemetryPass.addData("Error", gyro.getHeading()+degrees);
            telemetryPass.addData("TurnRight", turnRight);
            telemetryPass.update();
        }
//Albert was dead due to HIV on 11/29/2017

        runRight(0);
        runLeft(0);

        telemetryPass.addData("Target Ticks", targetTicks);
        telemetryPass.addData("Left Encoder Ticks", getLeftEncoder());
        telemetryPass.addData("Right Encoder Ticks", getRightEncoder());
        telemetryPass.addData("Gyro", gyro.getHeading());
        telemetryPass.update();
    }

    public void turnWithoutPID(int degrees, double timeout, double power, LinearOpMode opMode) throws InterruptedException {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetEncoders();
        gyro.reset();
        double startTime = System.currentTimeMillis();
        double targetTicks = degreesToTicks(Constants.ROBOT_TRACK*degrees/Constants.WHEEL_DIAMETER);
        double rightPower = power;
        double leftPower = power;
        boolean reachedTarget = false;
        double gyroError;

        if (degrees > 0){
            leftPower *= -1; //.93
        }

        else {
            rightPower *= -1;
        }

        while(System.currentTimeMillis() - startTime <= timeout*1000 && opMode.opModeIsActive() && reachedTarget == false){

            reachedTarget = ((Math.abs(getLeftEncoder()) > Math.abs(targetTicks)) && (Math.abs(getRightEncoder()) > Math.abs(targetTicks)));

            runRight(rightPower);
            runLeft(leftPower);

            telemetryPass.addData("Target Ticks", targetTicks);
            telemetryPass.addData("Left Encoder Ticks", getLeftEncoder());
            telemetryPass.addData("Right Encoder Ticks", getRightEncoder());
            telemetryPass.addData("Gyro", gyro.getHeading());
            telemetryPass.update();
        }
        runRight(0);
        runLeft(0);

        Thread.sleep(2000);

        while(System.currentTimeMillis() - startTime <= timeout*1000 && opMode.opModeIsActive()){

            gyroError = degrees - gyro.getHeading();

            if (Math.abs(gyroError) <= 0.5){break;}

            if (gyroError > 0){
                runLeft(-0.000001); //the reason the motor values are switched
                runRight(0.000001);
                Thread.sleep(100);
            }
            else {
                runLeft(0.000001);
                runRight(-0.000001);
                Thread.sleep(100);
            }
            telemetryPass.addData("Gyro", gyro.getHeading());
            telemetryPass.update();

        }

        runRight(0);
        runLeft(0);

        telemetryPass.addData("Target Ticks", targetTicks);
        telemetryPass.addData("Left Encoder Ticks", getLeftEncoder());
        telemetryPass.addData("Right Encoder Ticks", getRightEncoder());
        telemetryPass.addData("Gyro", gyro.getHeading());

        telemetryPass.update();
    }

    public boolean isBusy (){
        return rightFront.isBusy() && rightBack.isBusy() && leftFront.isBusy() && leftBack.isBusy();
    }

    public void turnEncoder(double degrees, double power, boolean right) {
        resetEncoders();
        if (right) {
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setTargetPosition((int)(degreesToTicks(degrees)/2));
            leftFront.setTargetPosition((int)(degreesToTicks(degrees)/2));
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            setPower(power);
            while (isBusy()) {
                telemetryPass.addData("left degrees", ticksToDegrees(getLeftEncoder()));
                telemetryPass.update();
            }
            setPower(0);
        } else {
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setTargetPosition((int) (degreesToTicks(degrees) / 2));
            rightFront.setTargetPosition((int) (degreesToTicks(degrees) / 2));
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            setPower(power);
            while (isBusy()) {
                telemetryPass.addData("right degrees", ticksToDegrees(getRightEncoder()));
                telemetryPass.update();
            }
            setPower(0);
        }
    }

}