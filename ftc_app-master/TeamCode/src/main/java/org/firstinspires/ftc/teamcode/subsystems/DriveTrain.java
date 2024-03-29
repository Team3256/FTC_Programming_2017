package org.firstinspires.ftc.teamcode.subsystems;

import android.hardware.camera2.CameraConstrainedHighSpeedCaptureSession;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.base.Constants;
import org.firstinspires.ftc.teamcode.base.PIDController;



/**
 * Created by Team 2891 on 9/25/2017.
 */

//Created by Team 2891

@TeleOp
public class DriveTrain {

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    private Servo rampServo;

    private static DriveTrain driveTrain = new DriveTrain();

    private HardwareMap hardwareMap;

    private IMUWrapper gyro;

    private DriveTrain(){

    }

    public void init(HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;

        gyro = new IMUWrapper("imu", hardwareMap);

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

        rampServo = hardwareMap.servo.get("rampServo");

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

        right = Range.clip(right, -1, 1)*.93;
        left = Range.clip(left, -1, 1);

        runLeft(left);
        runRight(right);

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
        PIDController distancePIDController = new PIDController(0.001, 0.0 , 0); //distance PID
        PIDController gyroPIDController = new PIDController(0.006, 0, 0);
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

            runRight(distancePID - gyroPID);
            runLeft(distancePID + gyroPID);

            /*telemetryPass.addData("Error",error);
            telemetryPass.addData("Distance PID: ", distancePID);
            telemetryPass.addData("Gyro", gyro.getHeading());
            telemetryPass.addData("Left encoder ticks",getLeftEncoder());
            telemetryPass.addData("Right encoder ticks",getRightEncoder());
            telemetryPass.addData("Inches",ticksToInches(getAverageEncoderValue()));
            telemetryPass.update(); */
        }

        runRight(0);
        runLeft(0);


        /*telemetryPass.addData("Right Front", rightFront.getCurrentPosition());
        telemetryPass.addData("Right Back", rightBack.getCurrentPosition());
        telemetryPass.addData("Left Front", leftFront.getCurrentPosition());
        telemetryPass.addData("Left Back", leftBack.getCurrentPosition());

        telemetryPass.addData("Loop","finished");
        telemetryPass.addData("Error",error);
        telemetryPass.addData("Gyro", gyro.getHeading());
        telemetryPass.addData("Left encoder ticks",getLeftEncoder());
        telemetryPass.addData("Right encoder ticks",getRightEncoder());
        telemetryPass.addData("Inches",degreesToInches(ticksToDegrees(getAverageEncoderValue())));
        telemetryPass.update(); */

        Thread.sleep(20);

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

            /*telemetryPass.addData("Target Ticks", targetTicks);
            telemetryPass.addData("Left Encoder Ticks", getLeftEncoder());
            telemetryPass.addData("Right Encoder Ticks", getRightEncoder());
            telemetryPass.update(); */
        }
        runRight(0);
        runLeft(0);

        Thread.sleep(20);

        if (degrees - gyro.getHeading() < 0){
            turnRight = true;
        }

        double turnPID;

        while(System.currentTimeMillis() - startTime <= timeout*1000 && opMode.opModeIsActive()){

            turnPID = turnPIDController.calculatePID(gyro.getHeading(), -degrees) * (turnRight ? 1 : -1);

            if ((Math.abs(gyro.getHeading() + degrees) <= 0.75)){
                break;
            }
            runRight(turnPID);
            runLeft(-turnPID);
            /*telemetryPass.addData("Target Ticks", targetTicks);
            telemetryPass.addData("Left Encoder Ticks", getLeftEncoder());
            telemetryPass.addData("Right Encoder Ticks", getRightEncoder());
            telemetryPass.addData("Gyro", gyro.getHeading());
            telemetryPass.addData("TurnPID", turnPID);
            telemetryPass.addData("Target", degrees);
            telemetryPass.addData("Error", gyro.getHeading()+degrees);
            telemetryPass.addData("TurnRight", turnRight);
            telemetryPass.update(); */
        }
        runRight(0);
        runLeft(0);

        /*telemetryPass.addData("Target Ticks", targetTicks);
        telemetryPass.addData("Left Encoder Ticks", getLeftEncoder());
        telemetryPass.addData("Right Encoder Ticks", getRightEncoder());
        telemetryPass.addData("Gyro", gyro.getHeading());
        telemetryPass.update(); */
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

            /*telemetryPass.addData("Target Ticks", targetTicks);
            telemetryPass.addData("Left Encoder Ticks", getLeftEncoder());
            telemetryPass.addData("Right Encoder Ticks", getRightEncoder());
            telemetryPass.addData("Gyro", gyro.getHeading());
            telemetryPass.update(); */
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
            //telemetryPass.addData("Gyro", gyro.getHeading());
            //telemetryPass.update();

        }

        runRight(0);
        runLeft(0);

        /*telemetryPass.addData("Target Ticks", targetTicks);
        telemetryPass.addData("Left Encoder Ticks", getLeftEncoder());
        telemetryPass.addData("Right Encoder Ticks", getRightEncoder());
        telemetryPass.addData("Gyro", gyro.getHeading()); */

        //telemetryPass.update();
    }

    public void driveRampDown () throws InterruptedException {
        rampServo.setPosition(1);
    }

    public void driveRampUp () throws InterruptedException {
        rampServo.setPosition(0.1);
    }

    public void driveRampMid (){
        rampServo.setPosition(0.5);
    }

    public boolean isBusy (){
        return rightFront.isBusy() && rightBack.isBusy() && leftFront.isBusy() && leftBack.isBusy();
    }
}
