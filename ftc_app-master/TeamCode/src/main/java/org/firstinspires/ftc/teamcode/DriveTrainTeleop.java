package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Glyph;

/**
 * Created by Team 3256 on 9/27/2017.
 */
@TeleOp
public class DriveTrainTeleop extends LinearOpMode {

    public static Telemetry telemetryPass;
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private Glyph glyph = Glyph.getInstance();
    private boolean running = false;


    @Override

    public void runOpMode() throws InterruptedException {
        DriveTrainTeleop.telemetryPass = telemetry;
        driveTrain.init(hardwareMap);
        glyph.init(hardwareMap);
        glyph.resetEncoders();

        float left, right;

        super.waitForStart();

        //Tank Drive

        while (opModeIsActive()){

            if(gamepad1.left_bumper) {
                if(!running) {
                    glyph.clampToggle();
                }
                running = true;
            } else {
                running = false;
            }

            /*if (gamepad1.a){
                glyph.elevatorDown();
            }

            else if (gamepad1.x){
                glyph.elevatorHalfUp();
            } */

            if (gamepad1.x){
                glyph.elevatorHalfUp(this);
            }

            else if (gamepad1.a){
                glyph.elevatorDown(this);
            }

            else if (gamepad1.y){
                glyph.elevatorFullUp(this);
            }

            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_x;
            if(Math.abs(gamepad1.left_stick_y) < 0.15){
                gamepad1.left_stick_y = 0;
            }
            if(Math.abs(gamepad1.right_stick_x) < 0.15){
                gamepad1.right_stick_x = 0;
            }

            //telemetry.addData("Left", left);
            //telemetry.addData("Right", right);
            driveTrain.arcadeDrive(left, right);
            //telemetry.addData("Left Encoder Value", driveTrain.getLeftEncoder());
            //telemetry.addData("Right Encoder Value", driveTrain.getRightEncoder());*/
            //telemetry.update();


        }


    }
}
