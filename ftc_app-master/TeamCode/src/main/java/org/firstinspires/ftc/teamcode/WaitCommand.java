package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Team 2891 on 10/31/2016.
 */
public class WaitCommand extends Command{

    private int wait_time;

    public WaitCommand(int wait_time){
        this.wait_time = wait_time;
    }

    public void initialize(HardwareMap hm) {

    }

    public void run() {
        try {
            Thread.sleep(wait_time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public boolean isFinished() {
        return true;
    }
}
