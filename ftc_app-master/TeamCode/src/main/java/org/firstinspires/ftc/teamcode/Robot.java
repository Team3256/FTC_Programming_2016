package org.firstinspires.ftc.teamcode;
import android.hardware.Sensor;

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
    public enum State{DISABLED, AUTONOMOUS, TELEOP}

    //Current state of Robot, starts out as Disabled
    State state = State.DISABLED;

    //ElapsedTime object for updating loop
    private ElapsedTime period  = new ElapsedTime();

    //DriveTrain
    private DriveTrain drive;

    //Intake
    private Intake intake;

    private SensorBase sensorBase;

    /**
     * Robot()
     * Empty Constructor
     */
    public Robot(){

    }

    /**
     * robotInit()
     * Initializes the entire robot for either autonomous or teleop, including all the subsystems
     * @param hm Instance of the HardwareMap of the robot
     * @param drive Instance of the DriveTrain of the robot
     * @param key Key to indicate what mode the robot should run
     */
    public void robotInit(HardwareMap hm, DriveTrain drive, Intake intake, SensorBase sensorBase, String key){
        this.drive=drive;
        this.intake = intake;
        this.sensorBase = sensorBase;
        this.hm=hm;
        if (key.equals("autonomous")){
            autonomousInit(hm,drive,intake);
        }
        if (key.equals("teleop")){
            teleopInit(hm,drive,intake,sensorBase);
        }
    }

    /**
     * autonomousInit()
     * Initializes robot for autonomous mode
     * @param hm Instance of the HardwareMap of the Robot
     * @param drive Instance of the DriveTrain of the Robot
     */
    public void autonomousInit(HardwareMap hm, DriveTrain drive, Intake intake){
        state = State.AUTONOMOUS;
        drive.init_Drive(hm,state);
        sensorBase.init_SensorBase(hm);
        //intake.init_Intake(hm);
        //sensorBase.resetSensors();
    }

    /**
     * teleopInit()
     * Initializes robot for teleop mode
     * @param hm Instance of the HardwareMap of the Robot
     * @param drive Instance of the DriveTrain of the Robot
     */
    public void teleopInit(HardwareMap hm, DriveTrain drive, Intake intake, SensorBase sensorBase){
        state = State.TELEOP;
        drive.init_Drive(hm,state);
        //sensorBase.init_SensorBase(hm);
        //intake.init_Intake(hm);
        sensorBase.init_SensorBase(hm);
        //sensorBase.resetSensors();
    }

    /**
     * waitForTick()
     * Waits until the next control loop tick starts
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
