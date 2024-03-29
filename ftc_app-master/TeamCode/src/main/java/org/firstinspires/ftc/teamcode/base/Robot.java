package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Beacon;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Robot {
    private HardwareMap hardwareMap;
    //time period for updating loop

    private ElapsedTime timePeriod = new ElapsedTime();

    //subsystems
    public DriveTrain driveTrain = DriveTrain.getInstance();
    public Shooter shooter = Shooter.getInstance();
    public Beacon beacon = Beacon.getBeacon();

    //singleton
    private static Robot robot = new Robot();

    private Robot() {

    }

    public void autonomousInit(HardwareMap hardwareMap){
        driveTrain.init(hardwareMap);
        shooter.init(hardwareMap);
        beacon.init(hardwareMap);
        beacon.initPos();
    }

    public void teleopInit(HardwareMap hardwareMap){
        driveTrain.init(hardwareMap);
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.init(hardwareMap);
        beacon.init(hardwareMap);
        beacon.setRightBangPos();
        beacon.setLeftBangPos();
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

    public double getAngle(){
        return driveTrain.getAngle();
    }

    public double getBlue(){
        return driveTrain.getBlue();
    }

    public double getRed() {
        return driveTrain.getRed();
    }

    public double getOds(){
        return driveTrain.getOds();
    }

    public boolean isBlue(){
        return driveTrain.isBlue();
    }

    public boolean gyroIsReady() {return driveTrain.gyroIsReady();}

    public double getPitch(){
        return driveTrain.getPitch();
    }
}
