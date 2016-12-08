package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "testMax")
public class TestMaxSpeed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = Robot.getInstance();
        robot.autonomousInit(hardwareMap);
        super.waitForStart();
        DriveTrain driveTrain = DriveTrain.getInstance();
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveTrain.setPower(1);
        long current = System.currentTimeMillis();
        while (System.currentTimeMillis() - current < 1000) {
            telemetry.addData("ticksLF", driveTrain.leftFront.getCurrentPosition());
            telemetry.addData("ticksRF", driveTrain.rightFront.getCurrentPosition());
            telemetry.addData("ticksLB", driveTrain.leftBack.getCurrentPosition());
            telemetry.addData("ticksRB", driveTrain.rightBack.getCurrentPosition());
            telemetry.update();
        }
        driveTrain.setPower(0);
        telemetry.addData("ticksLF", driveTrain.leftFront.getCurrentPosition());
        telemetry.addData("ticksRF", driveTrain.rightFront.getCurrentPosition());
        telemetry.addData("ticksLB", driveTrain.leftBack.getCurrentPosition());
        telemetry.addData("ticksRB", driveTrain.rightBack.getCurrentPosition());
        telemetry.update();
        sleep(10000);
    }
}
