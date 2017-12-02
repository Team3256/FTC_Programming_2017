package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Glyph;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;

import static android.R.attr.dropDownVerticalOffset;
import static android.R.attr.y;

/**
 * Created by Team 3256 on 9/27/2017.
 */
@TeleOp
public class DriveTrainTeleop extends LinearOpMode {

    public static Telemetry telemetryPass;
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private Glyph glyph = Glyph.getInstance();
    private Jewel jewel = Jewel.getInstance();
    private boolean runningClamp = false;
    private enum ElevatorState {
        DOWN, HALF, UP
    }
    private ElevatorState elevatorState;


    @Override

    public void runOpMode() throws InterruptedException {
        DriveTrainTeleop.telemetryPass = telemetry;
        driveTrain.init(hardwareMap);
        glyph.init(hardwareMap);
        jewel.init(hardwareMap);
        //jewel.resetArm();
        glyph.resetEncoders();

        float left, right;

        elevatorState = ElevatorState.DOWN;
        super.waitForStart();

        //Tank Drive

        while (opModeIsActive()){

            if(gamepad2.left_bumper) {
                if(!runningClamp) {
                    glyph.clampToggle();
                }
                runningClamp = true;
            } else {
                runningClamp = false;
            }

            if (gamepad2.a){
                //elevatorState = ElevatorState.DOWN;
                glyph.goUp();
            }

            /*
            else if (gamepad2.x){
                //elevatorState = ElevatorState.HALF;
            }*/

            else if (gamepad2.y) {
                glyph.goDown();
                //elevatorState = ElevatorState.UP;
            }

            else {
                glyph.stopElevator();
            }

            /*
            if (elevatorState == ElevatorState.DOWN){
                glyph.elevatorDown();
            }

            else if (elevatorState == ElevatorState.HALF){
                glyph.elevatorHalfUp();
            }

            else if (elevatorState == ElevatorState.UP){
                glyph.elevatorFullUp();
            }*/

            /*if (gamepad2.b){
                jewel.setArm();
            }

            if (gamepad2.dpad_up){
                jewel.jewelUp();
            }

            if (gamepad2.dpad_down){
                jewel.jewelDown();
            }

            if (gamepad2.start){
                telemetry.addData("Blue", jewel.isBlue());
                telemetry.update();
            } */

            if (gamepad1.right_bumper){
                glyph.intake();
            }

            else if (gamepad1.left_bumper){
                glyph.outtake();
            }

            else if (gamepad1.x){
                glyph.intakeOut();
            }

            else {
                glyph.zeroPower();
            }
            if (gamepad1.b){
                driveTrain.driveRampUp();
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
