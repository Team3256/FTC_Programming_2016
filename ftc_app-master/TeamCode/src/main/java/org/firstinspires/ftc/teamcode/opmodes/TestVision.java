package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;

/**
 * Created by Team 6696 on 12/3/2016.
 */
@Autonomous(name = "Vision")
public class TestVision extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        super.waitForStart();
        TelemetryHolder.telemetry = telemetry;
        VisionProcessor visionProcessor = new VisionProcessor();
        visionProcessor.init();
        while(opModeIsActive()){
            //visionProcessor.process(FtcRobotControllerActivity.getVisionFrame());
        }
    }
}
