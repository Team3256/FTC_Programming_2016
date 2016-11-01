package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Team 2891 on 10/31/2016.
 */
public class Beacon {

    private HardwareMap hm;
    private Servo leftDonger, rightDonger;

    public void init_Beacon(HardwareMap hm){
        this.hm = hm;
        leftDonger = hm.servo.get("leftDonger");
        rightDonger = hm.servo.get("rightDonger");
    }


}
