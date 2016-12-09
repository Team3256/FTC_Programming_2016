package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Passthroughs {
    public static Telemetry telemetry;
    public static LinearOpMode opMode;

    public static boolean opModeIsActive() {
        return opMode == null || opMode.opModeIsActive();
    }
}
