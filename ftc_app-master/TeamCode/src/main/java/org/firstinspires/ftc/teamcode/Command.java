package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 2891 on 10/28/2016.
 */
public abstract class Command {
    public abstract void initialize(HardwareMap hm);
    public abstract void run();
}
