package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Beacon;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Created by Team 6696 on 11/11/2016.
 */
public class Robot {
    private HardwareMap hardwareMap;

    public enum State {
        DISABLED,
        AUTONOMOUS,
        TELEOP
    }
    //state of the robot
    private State state = State.DISABLED;
    //time period for updating loop
    private ElapsedTime timePeriod = new ElapsedTime();
    //subsystems
    private DriveTrain driveTrain;
    private Intake intake;
    private SensorBase sensorBase;
    private Beacon beacon;
    //singleton
    private static Robot robot = new Robot();

    private Robot() {

    }

    public void robotInit(HardwareMap hardwareMap, DriveTrain driveTrain, Intake intake, Beacon beacon, SensorBase sensorBase, String key){
        this.hardwareMap = hardwareMap;
        if (key.equals("autonomous"))
            autonomousInit(hardwareMap, driveTrain, intake, beacon, sensorBase);
        if (key.equals("teleop"))
            teleopInit(hardwareMap, driveTrain, intake, beacon, sensorBase);
    }

    public void autonomousInit(HardwareMap hardwareMap, DriveTrain drive, Intake intake, Beacon beacon, SensorBase sensorBase){
        state = State.AUTONOMOUS;
        drive.initDrive(hardwareMap, state);
        sensorBase.initSensorBase(hardwareMap);
        //intake.init_Intake(hardwareMap);
        sensorBase.resetSensors();
        beacon.initBeacon(hardwareMap);
        beacon.initPos();
    }

    public void teleopInit(HardwareMap hardwareMap, DriveTrain drive, Intake intake, Beacon beacon, SensorBase sensorBase){
        state = State.TELEOP;
        drive.initDrive(hardwareMap, state);
        //intake.init_Intake(hm);
        beacon.initBeacon(hardwareMap);
        beacon.initPos();
        sensorBase.initSensorBase(hardwareMap);
        sensorBase.resetSensors();
    }

    public void waitForTick(long periodMs) throws InterruptedException {
        long remaining = periodMs - (long) timePeriod.milliseconds();
        if (remaining > 0)
            Thread.sleep(remaining);
        timePeriod.reset();
    }

    public static Robot getInstance() {
        return robot;
    }
}
