package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.ar.pl.DrawOverlayView;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import static com.sun.tools.javac.main.Option.D;

/**
 * Created by Team 3256 on 9/27/2017.
 */
@TeleOp
public class DriveTrainTest extends LinearOpMode {

    DriveTrain driveTrain = DriveTrain.getInstance();

    double left = 0, right = 0;


    @Override

    public void runOpMode() throws InterruptedException {

        driveTrain.init(hardwareMap);

        super.waitForStart();

        //Tank Drive
        /*
        while (opModeIsActive()){

            right = -gamepad1.left_stick_y;
            left = -gamepad1.right_stick_y;
            telemetry.addData("Left", left);
            telemetry.addData("Right", right);
            driveTrain.tankDrive(left, right);
            telemetry.update();

        }
        */

        //Arcade Drive
        while (opModeIsActive()){
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_x;
            telemetry.addData("Left", left);
            telemetry.addData("Right", right);
            driveTrain.arcadeDrive(left, right);

        }
    }
}