package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.base.SensorBase;
import org.firstinspires.ftc.teamcode.subsystems.Beacon;

/**
 * Created by Team 2891 on 11/9/2016.
 */
public class ServoSetPosition extends Command {
    SensorBase sensorBase = SensorBase.getInstance();
    Beacon beacon = Beacon.getBeacon();
    private boolean seeBlue;
    @Override
    public void initialize(HardwareMap hardwareMap) {
        sensorBase.initSensorBase(hardwareMap);
        sensorBase.resetSensors();
        beacon.initBeacon(hardwareMap);
    }

    private int counter = 0;
    @Override
    public void run() {
        counter++;
        if (seeBlue){
            beacon.setRightBangPos();
            beacon.setLeftNeutralPos();
        }
        else {
            beacon.setLeftBangPos();
            beacon.setRightNeutralPos();
        }
    }

    public void seeBlue(boolean seeBlue){
        this.seeBlue = seeBlue;
    }

    @Override
    public boolean isFinished() {
        return counter>=1;
    }
}
