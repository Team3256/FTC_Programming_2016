package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Robot;

@Autonomous(name = "AutoBlueBeacons")
public class AutoBlueBeacons extends LinearOpMode{
    private Robot robot = Robot.getInstance();
    
    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryHolder.telemetry = telemetry;
        robot.autonomousInit(hardwareMap);
        while(!robot.gyroIsReady()){
            telemetry.addData("gyro ready", "no");
            telemetry.addData("blueval", robot.getBlue());
            telemetry.update();
        }
        telemetry.addData("gyro ready", "yes");
        telemetry.addData("color sens", robot.getBlue());
        telemetry.addData("ods", robot.getOds());
        telemetry.update();
        //wait for play button
        super.waitForStart();
        //one wheel turn towards beacon
        robot.driveTrain.oneWheelTurn(34, 0.275, true);
        //drive to white line
        robot.driveTrain.driveToLine(60, 0.9);
        //drive a tiny bit forward so we are centered when we turn
        robot.driveTrain.driveToDistance(2.75, 0.4, true);
        //turn to the beacon
        robot.driveTrain.turn(37, 0.25, true);
        //set servo position depending on what color we see
        robot.beacon.updateServoPositionBlue();
        //hit beacon
        robot.driveTrain.driveToDistance(8, 0.35, true);
        //drive backward so we can turn towards second beacon
        robot.driveTrain.driveToDistance(6, 0.5, false);
        //reset servo positions to neutral for both
        robot.beacon.initPos();
        //turn towards the second beacon
        robot.driveTrain.turn(77, 0.4, false);
        //drive forward to prepare to see to the line
        robot.driveTrain.driveToDistance(40, 0.8, true);
        //drive forward to the line
        robot.driveTrain.driveToLine(12, 0.6);
        //drive forward a tiny bit so we are centered when we turn
        robot.driveTrain.driveToDistance(2.5, 0.4, true);
        //turn to the beacon
        robot.driveTrain.turn(80,0.3,true);
        //set servo positions depending on what we see
        robot.beacon.updateServoPositionBlue();
        //hit beacon
        robot.driveTrain.driveToDistance(8,0.35,true);
    }
}
