package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.ar.pl.DrawOverlayView;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import static android.R.attr.flipInterval;
import static android.R.attr.left;
import static android.R.attr.right;
import static com.sun.tools.javac.main.Option.D;

/**
 * Created by Team 3256 on 9/27/2017.
 */
@Autonomous
public class DriveTrainTest extends LinearOpMode {

    public static Telemetry telemetryPass;
    private DriveTrain driveTrain = DriveTrain.getInstance();


    @Override

    public void runOpMode() throws InterruptedException {
        DriveTrainTest.telemetryPass = telemetry;
        driveTrain.init(hardwareMap);

        float left, right;

        super.waitForStart();

        //Tank Drive

        //while (opModeIsActive()){

            /*right = -gamepad1.left_stick_y;
            left = -gamepad1.right_stick_y;
            telemetry.addData("Left", left);
            telemetry.addData("Right", right);
            driveTrain.tankDrive(left, right);
            telemetry.update();

        }

        */


        //Arcade Drive
            //
            /*left = gamepad1.left_stick_y;
            right = -gamepad1.right_stick_x;
            telemetry.addData("Left", left);
            telemetry.addData("Right", right);
            driveTrain.arcadeDrive(left, right); */

        driveTrain.driveToDistance(10, true, 30, this);

        //}
    }
}
