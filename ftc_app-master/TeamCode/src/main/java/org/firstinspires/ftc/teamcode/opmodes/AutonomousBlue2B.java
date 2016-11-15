package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Team 6696 on 11/11/2016.
 */
@Autonomous(name = "AutonomousBlue2B", group = "Linear OpMode")
public class AutonomousBlue2B extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoSequenceBlue2B sequence = new AutoSequenceBlue2B();
        sequence.initialize(hardwareMap);
        super.waitForStart();

        while (opModeIsActive()) {
            sequence.run();
            telemetry.addData("gyro", sequence.sensorBase.getAngle());
            telemetry.addData("step", sequence.currentStep);
            telemetry.update();
        }
    }
}
