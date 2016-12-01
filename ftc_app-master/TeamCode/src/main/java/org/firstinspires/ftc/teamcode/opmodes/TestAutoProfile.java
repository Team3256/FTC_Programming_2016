package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.trajectory.DriveController;

/**
 * Created by Team 2891 on 11/30/2016.
 */
@Autonomous(name="Profile")

public class TestAutoProfile extends LinearOpMode {
    private Robot robot = Robot.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryHolder.telemetry = telemetry;
        robot.autonomousInit(hardwareMap);
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveController autoProfileController = new DriveController();

        autoProfileController.setSetpoint(24);
        autoProfileController.init();

        super.waitForStart();

        autoProfileController.start();
        if (autoProfileController.isFinished()){
            telemetry.addData("DONEEEEEE", " ");
            autoProfileController.stop();
        }
        while(opModeIsActive()){

        }
    }
}
