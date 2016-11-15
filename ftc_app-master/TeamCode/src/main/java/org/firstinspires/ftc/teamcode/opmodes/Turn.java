package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.SensorBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

/**
 * Created by Team 6696 on 11/14/2016.
 */
public class Turn extends Command {
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private SensorBase sensorBase = SensorBase.getInstance();

    private double direction, setpoint, output;

    public Turn(HardwareMap hardwareMap, double output, double setpoint, boolean right) {
        this.initialize(hardwareMap);
        sensorBase.initSensorBase(hardwareMap);
        this.setpoint = setpoint;
        this.output = output;
        this.direction = right ? 1 : -1;
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sensorBase.resetSensors();
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {

    }

    @Override
    public void run() {
        driveTrain.runLeft(direction*-output);
        driveTrain.runRight(direction*output);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(setpoint - Math.abs(sensorBase.getAngle())) < 2;
    }

    @Override
    public void end() {
        driveTrain.runLeft(0);
        driveTrain.runRight(0);
    }
}
