package org.firstinspires.ftc.teamcode.base;

public class Constants {
    public static final double GEAR_RATIO = 2.33/1.0;
    public static final int GYRO_I2C_PORT = 1;
    public static final byte NAVX_GYRO_UPDATE_HZ = 50;
    public static final double WHEEL_DIAMETER = 4; //in inches
    public static final double TICKS_PER_ROTATION = 1120; //am 40
    public static final double ROBOT_TRACK = 15.153; //inches

    public static final double CLOSE_SHOT_PID_POWER = 0.40; //percent of max velocity (38000ticks/sec)

    public static final double AUTO_SHOOT_POWER = 0.40; //percent of max velocity same as close shot

    public static final double AUTO_BALL_TIME = 5; //seconds

    public static final double HOLD_BALL_SERVO_POS = 0.6;
    public static final double RELEASE_BALL_SERVO_POS = 1;

    public static final double kV = 1.0/20.0;
    public static final double kA = 0;

    public static final double kP = 0.0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double dt = 15.0/100.0;
}
