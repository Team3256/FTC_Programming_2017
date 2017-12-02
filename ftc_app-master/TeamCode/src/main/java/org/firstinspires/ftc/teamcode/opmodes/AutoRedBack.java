package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Glyph;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;
import org.firstinspires.ftc.teamcode.subsystems.VuforiaWrapper;

/**
 * Created by Team 3256 on 11/30/2017.
 */

@Autonomous(name = "Auto Red Back")
public class AutoRedBack extends LinearOpMode {


    private Jewel jewel = Jewel.getInstance();
    private Glyph glyph = Glyph.getInstance();
    private DriveTrain driveTrain = DriveTrain.getInstance();

    private String pictograph;

    @Override
    public void runOpMode() throws InterruptedException {
        jewel.init(hardwareMap);
        glyph.init(hardwareMap);
        driveTrain.init(hardwareMap);
        VuforiaWrapper vuforiaWrapper = new VuforiaWrapper(hardwareMap);
        vuforiaWrapper.initialize();

        waitForStart();

        if(jewel.isBlue()) {
            telemetry.addData("iS","bLue");
        } else {
            telemetry.addData("iS","rEd");
        }



        pictograph = vuforiaWrapper.getPictograph().toLowerCase();

        telemetry.addData("pictograph",pictograph);
        telemetry.update();

    }
}
