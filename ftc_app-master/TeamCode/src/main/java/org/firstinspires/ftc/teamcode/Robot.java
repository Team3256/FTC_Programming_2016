package org.firstinspires.ftc.teamcode;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Team 2891 on 9/16/2016.
 */
public class Robot{

    // Hardware Map to store all our electrical component objects
    private HardwareMap hm;

    //Drive Motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    //Navx micro gyro
    private AHRS gyro;

    //ElapsedTime object for updating loop
    private ElapsedTime period  = new ElapsedTime();

    //Device Interface Board
    private DeviceInterfaceModule dim;

    //I2C port on Device Interface Module for the navx micro gyro
    private final int GYRO_I2C_Port = 0;

    //Constructor
    public Robot(){

    }

    public void teleop_init(HardwareMap hm){
        //initialize the hardware map
        this.hm = hm;

        //initialize the motors
        leftFront = hm.dcMotor.get("leftFront");
        leftBack = hm.dcMotor.get("leftBack");
        rightFront = hm.dcMotor.get("rightFront");
        rightBack = hm.dcMotor.get("rightBack");

        //set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //set motor mode
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //initialize device interface module
        dim = hm.deviceInterfaceModule.get("Sensor Board");
        //initialize navx gyro
        gyro = AHRS.getInstance(dim, GYRO_I2C_Port, AHRS.DeviceDataType.kProcessedData);
    }

    public double getAngle(){
        return gyro.getRawGyroX();
    }
    public void waitForTick(long periodMs)  throws InterruptedException {
        long  remaining = periodMs - (long)period.milliseconds();
        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);
        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
