package org.firstinspires.ftc.teamcode.prototypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Glyph;

/**
 * Created by Team 3256 on 11/9/2017.
 */

@TeleOp

public class encoderTest extends LinearOpMode {

    private DriveTrain driveTrain = DriveTrain.getInstance();
    private Glyph glyph = Glyph.getInstance();
    public static Telemetry telemetryPass;

    @Override

    public void runOpMode() throws InterruptedException {

        encoderTest.telemetryPass = telemetry;

        driveTrain.init(hardwareMap);
        glyph.init(hardwareMap);

        super.waitForStart();

        while (opModeIsActive()){

            //telemetryPass.addData("Encoder:", driveTrain.getAverageEncoderValue());
            /*telemetryPass.addData("Right Encoder", driveTrain.getRightEncoder());
            telemetryPass.addData("Left Encoder", driveTrain.getLeftEncoder());
            telemetryPass.addData("Inches", Double.toString(driveTrain.ticksToInches(driveTrain.getAverageEncoderValue())));
            */
            telemetryPass.update();
        }

    }
}
