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
        Shooter shooter = Shooter.getInstance();
        shooter.setFlyRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.runFly(1);
        long current = System.currentTimeMillis();
        while (System.currentTimeMillis() - current < 1000) {
            telemetry.addData("ticks", shooter.getLEnc() + " " + shooter.getREnc());
            telemetry.update();
        }
        shooter.runFly(0);
        telemetry.addData("ticks", shooter.getLEnc() + " " + shooter.getREnc());
        telemetry.update();
        sleep(10000);
    }
}
