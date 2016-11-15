package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.base.SensorBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

/**
 * Created by Team 6696 on 11/14/2016.
 */
public class DriveForward extends Command {
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private SensorBase sensorBase = SensorBase.getInstance();

    private double power;

    public DriveForward(HardwareMap hardwareMap, double power, int ticks) {
        this.initialize(hardwareMap);
        this.power = power;
        driveTrain.setTargetPos(ticks);
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        driveTrain.initDrive(hardwareMap, Robot.State.AUTONOMOUS);
        driveTrain.resetEncoders();
        sensorBase.initSensorBase(hardwareMap);
    }

    @Override
    public void run() {
        driveTrain.runLeft(power);
        driveTrain.runRight(power);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driveTrain.ticksToInches(driveTrain.getTargetPos()) - Math.abs(driveTrain.ticksToInches((driveTrain.getRightEncoderValue() + driveTrain.getLeftEncoderValue()) / 2))) <= 2;
    }

    @Override
    public void end() {
        driveTrain.runLeft(0);
        driveTrain.runRight(0);
    }
}
