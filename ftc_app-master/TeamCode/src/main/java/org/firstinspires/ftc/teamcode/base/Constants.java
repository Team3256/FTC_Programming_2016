package org.firstinspires.ftc.teamcode.base;

public class Constants {
    public static final int GYRO_I2C_PORT = 1;
    public static final byte NAVX_GYRO_UPDATE_HZ = 50;
    public static final double WHEEL_DIAMETER = 4; //in inches
    public static final double TICKS_PER_ROTATION = 1120; //am 40

    public static final double CLOSE_SHOT_PID_POWER = 0.40; //percent of max velocity (38000ticks/sec)

    public static final double FAR_SHOT_PID_Power = 0.40; //percent of max velocity

    public static final double kV = 1.0/28.0;
    public static final double kA = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double dt = 5.0/100.0;
}
