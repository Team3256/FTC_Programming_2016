package org.firstinspires.ftc.teamcode;

/**
 * Created by Team 2891 on 9/17/2016.
 */
public class Constants {

    public static final int GYRO_I2C_PORT = 1;
    public static final byte NAVX_GYRO_UPDATE_HZ = 50;
    public static final double DRIVE_DISTANCE = 24;
    public static final double WHEEL_DIAMETER = 4;
    public static final double TICKS_PER_ROTATION = 1120;

    public static final double kP_STRAIGHT = 0.05;
    public static final double kI_STRAIGHT = 0;
    public static final double kD_STRAIGHT = 0.0;
    public static final double MIN_STRAIGHT_POWER = -0.55;
    public static final double MAX_STRAIGHT_POWER = 0.55;

    public static final double AUTO_TURN_ANGLE = 40;
    public static final double AUTO_TURN_ANGLE_2 = 19;
    public static final double MIN_TURN_OUTPUT = -0.75;
    public static final double MAX_TURN_OUTPUT = 0.75;
    public static final double kP_TURN = 0.008;
    public static final double kI_TURN = 0;
    public static final double kD_TURN = 0;
}
