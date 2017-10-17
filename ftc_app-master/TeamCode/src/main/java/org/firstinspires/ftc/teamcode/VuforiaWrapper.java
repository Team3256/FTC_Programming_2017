package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Team 3256 on 10/9/2017.
 */

public class VuforiaWrapper {

    public static final String TAG = "Vuforia VuMark Sample";

    VuforiaLocalizer vuforia;

    private HardwareMap hardwareMap;

    private VuforiaTrackable relicTemplate;

    public VuforiaWrapper (HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;
    }

    public void initialize (){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AbwIuA3/////AAAAGT2RK2RQL0bgqXiMpDxGldoFRJL3IzFDTI+xN+4FbSpol0Ukkf4gZgsOyPUCMZlEBGUlqOUbhGqpDD619vnFAkCPjg8Pmpm7/yEGmA3W8Icsssj42z5SiwsQrcYRmUiWlQGRW/3PKuTTNnFqCx4SI7+5d9XF30bEyB759km9S87tQNEgRixfn90Ci8GSqcx2IS9K4INhPWiNaxZd3Hx238E4eYFyZmSlResSHrRpTCRND/YRle9AQwSbV4EwiyB553rFRO2wnhIsbRfk6U7wA+sQxs1RBKBoOms1BEs2g46OAAfhEBA5uLlvuhLvV1OBEmz87LhuiAkjbuPrc84CjgiS2r9+6wMiVON4+MAIdytf";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();
    }

    public String getPictograph (){

        return RelicRecoveryVuMark.from(relicTemplate).name();
    }

}

