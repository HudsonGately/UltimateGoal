package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    /*** Intake Constants ***/
    public static double INTAKE_SPEED = 0.8;
    public static double OUTAKE_SPEED = -0.5;
    public static double SERVO_POSITION_SHOOT = 0.2;
    public static double SERVO_POSITION_HOME = 0.31;


    /*** Drive Constants ***/
    public static double DRIVE_TPR = 383.6;
    public static double DRIVE_WHEEL_DIAMETER = 2.3846;
    public static int DRIVE_STRAIGHT_P = 0;
    public static int DRIVE_STRAIGHT_D = 0;
    public static int DRIVE_GYRO_P = 0;
    public static int DRDIVE_GYRO_D = 0;
    /** Shooter Constants **/

    public static double MAX_SHOOTER_RPM = 4500;
    public static int SHOOTER_WHEEL_DIAMETER = 4;
    public static double SHOOTER_TPR = 28;
    public static double SHOOTER_OFFSET_ANGLE = 30.4;
    public static double ANGLER_TPR = 5264;

    public static double SHOOTER_P = 0.0002;
    public static double SHOOTER_F = 1.0 / MAX_SHOOTER_RPM;

    public static double ANGLER_P = 0;
    public static double GRAV_FF = 0;
    public static double ANGLER_D = 0;

    public static double TARGET_SPEED = 4500;

}
