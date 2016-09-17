package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Team 2891 on 9/16/2016.
 */
public class Robot{

    //HardwareMap of whole Robot
    private HardwareMap hm;

    //Enum for robot state
    public enum State{DISABLED, AUTONOMOUS, TELEOP};

    //Current state of Robot, starts out as Disabled
    State state = State.DISABLED;

    //ElapsedTime object for updating loop
    private ElapsedTime period  = new ElapsedTime();

    //DriveTrain
    private DriveTrain drive;

    //SensorBase
    private SensorBase sensorBase;

    /**
     * Robot()
     * @param hm Instance of the HardwareMap
     * @param drive Instance of the DriveTrain
     * @param sensorBase Instance of the SensorBase
     * @param key Key for what mode the robot should run in
     */
    public Robot(HardwareMap hm, DriveTrain drive, SensorBase sensorBase, String key){
        this.drive=drive;
        this.sensorBase=sensorBase;
        if (key == "autonomous"){
            autonomousInit(hm);
        }
        if (key == "teleop"){
            teleopInit(hm);
        }
    }

    /**
     * autonomousInit()
     * Initializes robot for autonomous mode
     * @param hm HardwareMap of the Robot
     */
    public void autonomousInit(HardwareMap hm){
        state = State.AUTONOMOUS;
        drive = new DriveTrain(hm, state);
        sensorBase = new SensorBase(hm);
        sensorBase.resetSensors();
    }

    /**
     * teleopInit()
     * Initializes robot for teleop mode
     * @param hm HardwareMap of the Robot
     */
    public void teleopInit(HardwareMap hm){
        state = State.TELEOP;
        drive = new DriveTrain(hm, state);
        sensorBase = new SensorBase(hm);
        sensorBase.resetSensors();
    }

    /**
     * waitForTick()
     * @param periodMs period in milliseconds of the control loop
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException {
        long  remaining = periodMs - (long)period.milliseconds();
        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);
        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
