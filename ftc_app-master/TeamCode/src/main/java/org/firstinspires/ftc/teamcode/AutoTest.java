package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Glyph;
import org.firstinspires.ftc.teamcode.subsystems.IMUWrapper;

/**
 * Created by Team 3256 on 11/10/2017.
 */

@Autonomous
public class AutoTest extends LinearOpMode{

    public static Telemetry telemetryPass;
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private Glyph glyph = Glyph.getInstance();


    @Override

    public void runOpMode() throws InterruptedException {
        AutoTest.telemetryPass = telemetry;
        driveTrain.init(hardwareMap);
        glyph.init(hardwareMap);



        super.waitForStart();

        //driveTrain.turnEncoder(45, .5, true);

        glyph.clampIn();

        driveTrain.driveToDistance(20, true, 30, this);
        glyph.clampOut();
        driveTrain.driveToDistance(2, false, 30, this);
        driveTrain.driveToDistance(1.5 , true, 30, this);
        //driveTrain.driveRamp();
        //driveTrain.driveToDistance(24, true, 30, this);

        //driveTrain.turnWithPID(90,30,1,this);
        //driveTrain.driveToDistance(18, true, 30, this);

        //driveTrain.turnWithPID(90,30,1,this);

        Thread.sleep(5000);
    }

}
