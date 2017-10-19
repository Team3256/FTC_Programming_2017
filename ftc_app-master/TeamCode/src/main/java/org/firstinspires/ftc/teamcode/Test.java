package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.VuforiaWrapper;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by Team 3256 on 9/20/2017.
 */
@TeleOp
public class Test extends LinearOpMode {

    DriveTrain driveTrain = DriveTrain.getInstance();

    double left = 0, right = 0;

    public static final String TAG = "Vuforia VuMark Sample";

    private boolean justPressed = true;

    @Override
    public void runOpMode() throws InterruptedException {

        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AbwIuA3/////AAAAGT2RK2RQL0bgqXiMpDxGldoFRJL3IzFDTI+xN+4FbSpol0Ukkf4gZgsOyPUCMZlEBGUlqOUbhGqpDD619vnFAkCPjg8Pmpm7/yEGmA3W8Icsssj42z5SiwsQrcYRmUiWlQGRW/3PKuTTNnFqCx4SI7+5d9XF30bEyB759km9S87tQNEgRixfn90Ci8GSqcx2IS9K4INhPWiNaxZd3Hx238E4eYFyZmSlResSHrRpTCRND/YRle9AQwSbV4EwiyB553rFRO2wnhIsbRfk6U7wA+sQxs1RBKBoOms1BEs2g46OAAfhEBA5uLlvuhLvV1OBEmz87LhuiAkjbuPrc84CjgiS2r9+6wMiVON4+MAIdytf";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0); */

        //DistanceSensor sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        //IMUWrapper imu = new IMUWrapper("imu", hardwareMap);
        VuforiaWrapper vuforiaWrapper = new VuforiaWrapper(hardwareMap);

        //driveTrain.init(hardwareMap);

        //imu.calibrate();
        vuforiaWrapper.initialize();

        super.waitForStart();

        //relicTrackables.activate();

        while (opModeIsActive()) {

            /*telemetry.addData("Distance (cm)", sensorColorRange.getDistance(DistanceUnit.CM));
            telemetry.addData("Color: Red", colorSensor.red());
            telemetry.addData("Color: Green", colorSensor.green());
            telemetry.addData("Color: Blue", colorSensor.blue());
            telemetry.update();
            */

/*
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_x;
            telemetry.addData("Left", left);
            driveTrain.arcadeDrive(left, right);


            if (gamepad1.right_bumper) {

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    telemetry.addData("VuMark", "%s visible", vuMark);

                } else {
                    telemetry.addData("VuMark", "not visible");
                }

                telemetry.update();

            }
            */

            /*if (justPressed) {

                justPressed = false;
            } else if (gamepad1.y){

                imu.reset();
                justPressed = true;
            }

            if (gamepad1.x) {
                imu.calibrate();
            }

            telemetry.addData("GetHeading", imu.getHeading());
            telemetry.addData("Angle", imu.angle);
            telemetry.addData("Offset", imu.offset);
            telemetry.update(); */

            if (gamepad1.x){
                telemetry.addData(vuforiaWrapper.getPictograph(), "");
            }

            telemetry.update();


        }

    }

}