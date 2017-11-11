package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

/**
 * Created by Team 3256 on 11/10/2017.
 */

@Autonomous
public class AutoTest extends LinearOpMode{

    public static Telemetry telemetryPass;
    private DriveTrain driveTrain = DriveTrain.getInstance();


    @Override

    public void runOpMode() throws InterruptedException {
        AutoTest.telemetryPass = telemetry;
        driveTrain.init(hardwareMap);


        super.waitForStart();

        driveTrain.driveToDistance(30, true, 30, this);


    }

}
