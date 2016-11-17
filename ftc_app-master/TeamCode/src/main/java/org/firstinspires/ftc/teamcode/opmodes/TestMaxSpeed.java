package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

/**
 * Created by Team 6696 on 11/16/2016.
 */
@Autonomous(name = "testMax")
public class TestMaxSpeed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = Robot.getInstance();
        robot.autonomousInit(hardwareMap);
        super.waitForStart();
        long current = System.currentTimeMillis();
        DriveTrain driveTrain = DriveTrain.getInstance();
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveTrain.setPower(1);
        while (System.currentTimeMillis() - current < 1000) {
            telemetry.addData("ticks", driveTrain.getLeftEncoderValue() + " " + driveTrain.getRightEncoderValue());
            telemetry.update();
        }
        driveTrain.setPower(0);
        telemetry.addData("ticks", driveTrain.getLeftEncoderValue() + " " + driveTrain.getRightEncoderValue());
        telemetry.update();
        sleep(10000);
    }
}
