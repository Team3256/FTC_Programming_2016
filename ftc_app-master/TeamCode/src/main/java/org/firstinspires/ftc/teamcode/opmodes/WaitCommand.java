package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmodes.Command;

/**
 * Created by Team 2891 on 10/31/2016.
 */
public class WaitCommand extends Command {
    private int waitTime;

    public WaitCommand(int waitTime){
        this.waitTime = waitTime;
    }

    public void initialize(HardwareMap hm) {

    }

    public void run() {
        try {
            Thread.sleep(waitTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public boolean isFinished() {
        return true;
    }

    @Override
    public void end() {

    }
}
