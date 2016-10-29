package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
 * Created by Team 2891 on 10/29/2016.
 */
@TeleOp(name="VuforiaTest")
public class VuforiaTest extends LinearOpMode {
    VuforiaLocalizer vuforia;
    VuforiaTrackables beacons;
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AcI/Uyj/////AAAAGXv7+ToHVk7nkyKsu5GaH3kglen0Hd02gmnb8hnFcqLYId2oCB+L1fLLNf10bowXS9YnZuqA3fQEO/pon3ZKIi4JgHg4SQrpX4G8K1Z+jHBUHStSlBHdmB9VIaCse3+96dSCEX/cWEVebbZYmM+bh7Nn60vNJLUAKGMHweAOPzX5+/wy0roGowuRi3JxEI/WiF/v2D2FLsM3UTHHZ9hUVwQGfFZaQpRtQr6e34nuFdqThSJP1LpsrUnk2IJFud5POaUz8sQmGTlSK2XB0rylkRbxsC06vRGUG9XFDvA3m7v9oohajc0HpZZlqRhgSAgQ5KkS8s91QCufaJkzTX597cHO9THjy23aODBawqJTFmNP";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        super.waitForStart();
        beacons.activate();

        while(opModeIsActive()){
            for(VuforiaTrackable beac:beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)beac.getListener()).getPose();

                if (pose!=null){
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beac.getName() + "-Translation", translation);
                    double degreestoTurn = Math.min(Math.abs(Math.toDegrees(Math.atan2(translation.get(0),translation.get(2)))+180),Math.abs(Math.toDegrees(Math.atan2(translation.get(0),translation.get(2)))-180));

                    telemetry.addData(beac.getName() + "-Degrees", degreestoTurn);

                }

            }
            telemetry.update();

        }
    }
}
