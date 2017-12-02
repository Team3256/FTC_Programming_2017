package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Glyph;
import org.firstinspires.ftc.teamcode.subsystems.Jewel;

/**
 * Created by Team 3256 on 11/30/2017.
 */

@Autonomous(name = "Auto Blue Front")

public class AutoBlueFront extends LinearOpMode {

    private Jewel jewel = Jewel.getInstance();
    private Glyph glyph = Glyph.getInstance();
    private DriveTrain driveTrain = DriveTrain.getInstance();

    private String pictograph;

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
