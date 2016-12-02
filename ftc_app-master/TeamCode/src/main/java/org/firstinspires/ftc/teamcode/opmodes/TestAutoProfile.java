package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.base.Constants;
import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
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
        double start = System.currentTimeMillis();
        robot.autonomousInit(hardwareMap);
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveTrain driveTrain = DriveTrain.getInstance();
        DriveController autoProfileController = new DriveController();

        autoProfileController.setSetpoint(24*2);
        autoProfileController.init();

        super.waitForStart();

        autoProfileController.start();
        if (autoProfileController.isFinished()){
            autoProfileController.stop();
        }

        while(opModeIsActive()){

        }
    }
}
