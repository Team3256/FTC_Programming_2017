package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Team 6696 on 9/27/2017.
 */

public class VuforiaOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ARJPFtb/////AAAAGSY6dQi9tk+XgBKbw6Qq6707wFdmmsMalJ3QlH2HULs4lf+/5J8yvdO//SNbu7dhQQSq4Z/r9zn/5xDvQm4gYzGBU2cB8wij4ZI/tMtMtJXvZIlr+K933mdYydYMnfq8yf8QA0tUUWE5t8oR21JLEqZuDMwQdVm/EzA0zjVabKKapj+Wyo76cCmp6HeqwngT4997AuElXAThizwRtPWwJHKGWKlVH/Ke3Ti9hoIlX2CvIh49w+OHa0eUXRVc103BR4Yy4sXofgDs7KbAlXWf3qPpn8jzVDq+jK0qosyLYkBpmH6xr+qxwTJxjB+/8C+WVN+EP6hPScLVHwZ4SpVocCnHHhWRbJ+q62y+0YjcLugu";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 3);

        VuforiaTrackables pictographs = vuforia.loadTrackablesFromAsset("RelicVuMark");
        pictographs.get(0).setName("Left Relic");
        pictographs.get(1).setName("Center Relic");
        pictographs.get(2).setName("Right Relic");

        super.waitForStart();

        pictographs.activate();
        while (opModeIsActive()){
            for (VuforiaTrackable pict : pictographs){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)pict.getListener()).getPose();

                if(pose != null){
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(pict.getName() + "-Translation", translation);

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(pict.getName() + "-Degrees", degreesToTurn);
                }
            }

            telemetry.update();
        }

    }
}
