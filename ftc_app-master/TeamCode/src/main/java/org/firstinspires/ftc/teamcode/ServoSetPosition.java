package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 2891 on 11/9/2016.
 */
public class ServoSetPosition extends Command {
    SensorBase sensorBase = new SensorBase();
    Beacon beacon = new Beacon();
    private boolean seeBlue;
    @Override
    public void initialize(HardwareMap hm) {
        sensorBase.init_SensorBase(hm);
        sensorBase.resetSensors();
        beacon.init_Beacon(hm);
    }

    private int counter = 0;
    @Override
    public void run() {
        counter++;
        if (seeBlue){
            beacon.setRightBangPos();
            beacon.setLeftNeutralPos();
        }
        else if (!seeBlue){
            beacon.setLeftBangPos();
            beacon.setRightNeutralPos();
        }
        else {
            beacon.setLeftNeutralPos();
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
