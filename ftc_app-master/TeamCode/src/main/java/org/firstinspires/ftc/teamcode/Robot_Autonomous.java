package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.kauailabs.navx.ftc.navXPIDController;

import java.text.DecimalFormat;

/**
 * Created by Team 2891 on 9/17/2016.
 */
@Autonomous(name="Autonmous", group="Linear OpMode")
public class Robot_Autonomous extends LinearOpMode{

    private DriveTrain drive = new DriveTrain();
    private SensorBase sensorBase = new SensorBase();
    private Intake intake = new Intake();

    private Robot robot = new Robot();

    navXPIDController turnController;

    public void runOpMode() throws InterruptedException {
        robot.robotInit(super.hardwareMap, drive,intake, "autonomous");
        turnController = new navXPIDController(sensorBase.gyro,navXPIDController.navXTimestampedDataSource.YAW);
        turnController.setSetpoint(Constants.AUTO_STATE1_TURN_DEGREES);
        turnController.setContinuous(true);
        turnController.setOutputRange(-1, 1);
        turnController.setPID(Constants.TURN_PID_KP, Constants.TURN_PID_KI, Constants.TURN_PID_KD);
        super.waitForStart();
        try {
            turnController.enable(true);
            int DEVICE_TIMEOUT_MS = 500;
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            DecimalFormat df = new DecimalFormat("#.##");

            while ( opModeIsActive() &&
                    !Thread.currentThread().isInterrupted()) {
                /*
                if (turnController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        drive.runLeft(0);
                        drive.runRight(0);
                    } else {
                        double output = yawPIDResult.getOutput();
                        drive.runLeft(output);
                        drive.runRight(-output);
                        telemetry.addData("PIDOutput", df.format(output) + ", " +
                                df.format(-output));
                    }
                    telemetry.addData("PIDOutput", df.format(0.00));

                }
                */
                telemetry.addData("Yaw", df.format(sensorBase.getAngle()));
                robot.waitForTick(40);
                idle();
            }

        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        finally {
            turnController.close();
            telemetry.addData("LinearOp", "Complete");
        }
    }
}
