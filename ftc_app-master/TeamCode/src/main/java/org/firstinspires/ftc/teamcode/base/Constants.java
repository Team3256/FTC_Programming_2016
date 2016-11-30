package org.firstinspires.ftc.teamcode.base;

public class Constants {
    public static final int GYRO_I2C_PORT = 1;
    public static final byte NAVX_GYRO_UPDATE_HZ = 50;
    public static final double WHEEL_DIAMETER = 4; //in inches
    public static final double TICKS_PER_ROTATION = 1120; //am 40

    public static final double CLOSE_SHOT_PID_POWER = 0.3; //percent of max velocity (38000ticks/sec)

    public static final double FAR_SHOT_PID_Power = 0.35; //percent of max velocity
}
