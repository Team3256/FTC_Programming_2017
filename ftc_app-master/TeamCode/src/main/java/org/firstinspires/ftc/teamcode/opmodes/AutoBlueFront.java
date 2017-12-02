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

@Autonomous(name = "Auto Blue Front")

public class AutoBlueFront extends LinearOpMode {

    private Jewel jewel = Jewel.getInstance();
    private Glyph glyph = Glyph.getInstance();
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private int random = 0;
    private int counter = 0;

    private String pictograph;

    @Override
    public void runOpMode() throws InterruptedException {
        jewel.init(hardwareMap);
        glyph.init(hardwareMap);
        driveTrain.init(hardwareMap);
        VuforiaWrapper vuforiaWrapper = new VuforiaWrapper(hardwareMap);
        vuforiaWrapper.initialize();

        waitForStart();

        glyph.clampIn();
        jewel.setArmDown();

        if (jewel.isBlue()) {
            telemetry.addData("is", "blue");
            driveTrain.driveToDistance(5.5, true, 5, this);
            while (pictograph == null && counter <= 100) {
                pictograph = vuforiaWrapper.getPictograph().toLowerCase();
                Thread.sleep(10);
                counter ++;
            }
            jewel.setArmUp();
            driveTrain.driveToDistance(5.5, false, 5, this);
        } else {
            telemetry.addData("is", "red");
            driveTrain.driveToDistance(3.5, false, 5, this);
            jewel.setArmUp();
            driveTrain.driveToDistance(9, true, 5, this);
            while (pictograph == null && counter <= 100) {
                pictograph = vuforiaWrapper.getPictograph().toLowerCase();
                Thread.sleep(10);
                counter ++;
            }
            driveTrain.driveToDistance(5.5, false, 5, this);
        }

        driveTrain.turnWithPID(180, 5, 0.5, this);

        if (pictograph == null){
            random = (int)(Math.random()*3 + 1);
            if (random == 1){
                pictograph = "left";
            }
            if (random == 2){
                pictograph = "right";
            }
            if (random == 3){
                pictograph = "center";
            }
        }

        switch (pictograph){
            case "left":
                break;
            case "center":
                driveTrain.driveToDistance(7, true, 7, this);
                break;
            case "right":
                driveTrain.driveToDistance(14, true, 10, this);
        }

        driveTrain.turnWithPID(-90, 5, 0.5, this);
        driveTrain.driveToDistance(7, true, 5, this);

        glyph.clampOut();

        driveTrain.driveToDistance(2, false, 5, this);
        driveTrain.driveToDistance(2.5, true, 5, this);
        telemetry.update();
    }
}
