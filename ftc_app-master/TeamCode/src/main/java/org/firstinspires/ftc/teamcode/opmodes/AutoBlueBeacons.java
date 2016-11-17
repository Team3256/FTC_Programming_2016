package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.Robot;

/**
 * Created by Team 6696 on 11/15/2016.
 */
@Autonomous(name = "AutoBlueBeacons")
public class AutoBlueBeacons extends LinearOpMode{

    private Robot robot = Robot.getInstance();
    private boolean beac1blue = false, beac2blue = false;
    public static Telemetry telemetryPass;
    
    @Override
    public void runOpMode() throws InterruptedException {
        telemetryPass = telemetry;
        robot.autonomousInit(hardwareMap);
        while(!robot.gyroIsReady()&&robot.getBlue()==255){
            telemetry.addData("gyro not ready", "");
            telemetry.addData("blueval ", robot.getBlue());
            telemetry.update();
        }
        telemetry.addData("gyro is ready", "");
        telemetry.addData("color sens is ready", robot.getBlue());
        telemetry.update();
        super.waitForStart();
        telemetry.addData("beac blue val", robot.getBlue());
        telemetry.addData("beaconisblue", robot.isBlue());
        robot.driveTrain.oneWheelTurn(31, 0.22, true);
        robot.driveTrain.driveToLine(60, 0.9);
        robot.driveTrain.driveToDistance(1, 0.4);
        robot.driveTrain.turn(38,0.15,true);
        beac1blue = robot.isBlue();
        robot.driveTrain.driveToDistance(3, 0.4);
        robot.beacon.setServoPosition(beac1blue);
        /*
        robot.driveTrain.driveToDistance(6,0.2);
        robot.beacon.initPos();
        robot.driveTrain.turn(80,0.15,false);
        robot.driveTrain.driveToDistance(40,0.5);
        robot.driveTrain.driveToLine(24,0.2);
        robot.driveTrain.turn(80,0.15,true);
        robot.driveTrain.driveToDistance(8,0.2);
        beac2blue = robot.isBlue();
        robot.driveTrain.driveToDistance(-6,0.2);
        robot.driveTrain.driveToDistance(6,0.2);*/
        sleep(10000);
    }
}
