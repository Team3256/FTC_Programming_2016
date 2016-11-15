package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.base.SensorBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

/**
 * Created by Team 6696 on 11/14/2016.
 */
public class OneWheelTurn extends Command {
    private double power, setpoint;
    private boolean right;

    private DriveTrain driveTrain = DriveTrain.getInstance();
    private SensorBase sensorBase = SensorBase.getInstance();

    public OneWheelTurn(HardwareMap hardwareMap, double power, double setpoint, boolean right) {
        this.initialize(hardwareMap);
        this.power = power;
        this.setpoint = setpoint;
        this.right = right;
        sensorBase.resetGyro();
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        driveTrain.initDrive(hardwareMap, Robot.State.TELEOP);
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sensorBase.initSensorBase(hardwareMap);
        sensorBase.resetSensors();
    }

    @Override
    public void run() {
        if (right)
            driveTrain.runRight(power);
        else driveTrain.runLeft(power);
    }

    @Override
    public boolean isFinished() {
        return setpoint - Math.abs(sensorBase.getAngle()) < 2;
    }

    @Override
    public void end() {
        if (right)
            driveTrain.runRight(0);
        else driveTrain.runLeft(0);
    }
}
