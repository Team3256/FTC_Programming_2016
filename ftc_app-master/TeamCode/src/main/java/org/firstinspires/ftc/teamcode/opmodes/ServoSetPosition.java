package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.SensorBase;
import org.firstinspires.ftc.teamcode.subsystems.Beacon;

/**
 * Created by Team 6696 on 11/14/2016.
 */
public class ServoSetPosition extends Command {
    private SensorBase sensorBase = SensorBase.getInstance();
    private Beacon beacon = Beacon.getBeacon();
    private boolean blueSeen;

    private boolean isFinished = false;

    public void setBlueSeen(boolean blueSeen) {
        this.blueSeen = blueSeen;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        sensorBase.initSensorBase(hardwareMap);
        sensorBase.resetSensors();
        beacon.initBeacon(hardwareMap);
    }

    @Override
    public void run() {
        isFinished = true;
        if (blueSeen) {
            beacon.setRightBangPos();
            beacon.setLeftNeutralPos();
        } else {
            beacon.setRightNeutralPos();
            beacon.setLeftBangPos();
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end() {

    }
}
