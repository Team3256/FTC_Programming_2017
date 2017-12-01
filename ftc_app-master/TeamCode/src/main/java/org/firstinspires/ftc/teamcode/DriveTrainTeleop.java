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

            if (gamepad1.left_bumper){
                if (running == false) {
                    glyph.clampToggle();
                    running = true;
                } else {
                    running = false;
                }
            }

            if (gamepad1.x){
                glyph.elevatorHalfUp(this);
            }
            else if (gamepad1.a){
                glyph.elevatorDown(this);
            }
            else if (gamepad1.y){
                glyph.elevatorFullUp(this);
            }

            /*
            if (gamepad2.left_bumper) {
                glyph.intake();
            }
            else if (gamepad2.right_bumper){
                glyph.outtake();
            }

            else {
                glyph.zeroPower();
            }
            */

            if (gamepad1.dpad_up){
                driveTrain.driveRampUp();
            }
            else if (gamepad1.dpad_down) {
                driveTrain.driveRampDown();
            }

            if (Math.abs(gamepad2.left_stick_y) > 0.15){
                glyph.manualElevator(gamepad2.left_stick_y);
            }

            if (gamepad2.start){
                glyph.resetEncoders();
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
