package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.base.Robot;

/**
 * Created by Team 2891 on 12/9/2016.
 */
@Autonomous(name = "test")
public class AutoTest extends LinearOpMode {
    private Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        Passthroughs.telemetry = telemetry;
        Passthroughs.opMode = this;
        robot.autonomousInit(hardwareMap);
        super.waitForStart();
        //two wheel turn test left
        //for (int i = 0; i < 2; i++) {
            //robot.driveTrain.driveToDistance(10,0.5,true);
            //robot.driveTrain.driveToDistance(10,0.5,false);
        //    sleep(250);
        //}
        //two wheel turn test right
        /*
        for (int i = 0; i < 2; i++) {
            robot.driveTrain.turn(45, .55, true, 1);
            sleep(250);
        }
        */
        /*
        for (int i = 0; i < 4; i++) {
            robot.driveTrain.oneWheelTurn(35, 0.44, false);
            sleep(250);
        }
        */
    }

}
