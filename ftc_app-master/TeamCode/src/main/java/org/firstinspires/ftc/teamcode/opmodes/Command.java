package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 6696 on 11/14/2016.
 */
public abstract class Command {
    public abstract void initialize(HardwareMap hardwareMap);
    public abstract void run();
    public abstract boolean isFinished();
    public abstract void end();
}
