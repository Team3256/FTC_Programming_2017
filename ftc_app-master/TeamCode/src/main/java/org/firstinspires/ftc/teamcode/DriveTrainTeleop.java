package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

/**
 * Created by Team 3256 on 9/27/2017.
 */
@TeleOp
public class DriveTrainTeleop extends LinearOpMode {

    public static Telemetry telemetryPass;
    private DriveTrain driveTrain = DriveTrain.getInstance();



    @Override

    public void runOpMode() throws InterruptedException {
        DriveTrainTeleop.telemetryPass = telemetry;
        driveTrain.init(hardwareMap);

        float left, right;

        super.waitForStart();

        //Tank Drive

        while (opModeIsActive()){


            left = gamepad1.left_stick_y;
            right = -gamepad1.right_stick_x;
            if(Math.abs(gamepad1.left_stick_y) < 0.15){
                gamepad1.left_stick_y = 0;
            }
            if(Math.abs(gamepad1.right_stick_x) < 0.15){
                gamepad1.right_stick_x = 0;
            }
            telemetry.addData("Left", left);
            telemetry.addData("Right", right);
            driveTrain.arcadeDrive(left, right);
            telemetry.addData("Left Encoder Value", driveTrain.getLeftEncoder());
            telemetry.addData("Right Encoder Value", driveTrain.getRightEncoder());
            telemetry.update();


        }


    }
}
