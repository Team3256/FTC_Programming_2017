package org.firstinspires.ftc.teamcode.subsystems;

import android.hardware.camera2.CameraConstrainedHighSpeedCaptureSession;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.Constant;
import org.firstinspires.ftc.teamcode.base.Constants;
import org.firstinspires.ftc.teamcode.base.PIDController;

import static com.sun.tools.javac.jvm.ByteCodes.error;
import static org.firstinspires.ftc.teamcode.AutoTest.telemetryPass;


/**
 * Created by Team 2891 on 9/25/2017.
 */
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
        double left = Math.pow((throttle + turn), 3);
        double right = Math.pow((throttle - turn), 3);

        right = Range.clip(right, -1, 1);
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

        telemetryPass.addData("Loop","finished");
        telemetryPass.addData("Error",error);
        telemetryPass.addData("Gyro", gyro.getHeading());
        telemetryPass.addData("Left encoder ticks",getLeftEncoder());
        telemetryPass.addData("Right encoder ticks",getRightEncoder());
        telemetryPass.addData("Inches",degreesToInches(ticksToDegrees(getAverageEncoderValue())));
        telemetryPass.update();

        Thread.sleep(20000);

    }

    /*
    public void turn(double degrees, double power, boolean right){

        int direction = right ? 1 : -1;
        PIDController turnPIDController = new PIDController(0, 0, 0);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro.reset();
    }
    */

    public boolean isBusy (){
        return rightFront.isBusy() && rightBack.isBusy() && leftFront.isBusy() && leftBack.isBusy();
    }

}