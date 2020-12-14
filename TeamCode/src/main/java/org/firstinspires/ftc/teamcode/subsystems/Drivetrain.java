package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;
import java.util.logging.Logger;
@Config
public class Drivetrain extends SubsystemBase {

    public static double DRIVE_TPR = 383.6;
    public static double DRIVE_WHEEL_DIAMETER = 2.3846;
    public static int DRIVE_STRAIGHT_P = 0;
    public static int DRIVE_STRAIGHT_D = 0;
    public static int DRIVE_GYRO_P = 0;
    public static int DRIVE_GYRO_D = 0;

    Telemetry telemetry;

    private DifferentialDrive drive;
    private RevIMU gyro;
    private Motor.Encoder left, right;
    public Drivetrain(MotorEx backLeft, MotorEx backRight, MotorEx frontLeft, MotorEx frontRight, RevIMU gyro, Motor.Encoder leftEncoder, Motor.Encoder rightEncoder, Telemetry tl) {
        this.gyro = gyro;
        this.gyro.init();
        this.gyro.invertGyro();

        drive = new DifferentialDrive(false, new MotorGroup(backLeft, frontLeft), new MotorGroup(backRight, frontRight));
        this.left = leftEncoder;
        this.right = rightEncoder;
        this.telemetry = tl;
    }

    public Drivetrain(MotorEx backLeft, MotorEx backRight, MotorEx frontLeft, MotorEx frontRight, RevIMU gyro, Telemetry tl) {
        this(backLeft, backRight, frontLeft, frontRight, gyro, backLeft.encoder, backRight.encoder, tl);
    }

    @Override
    public void periodic() {
        Util.logger(this, telemetry, Level.INFO, "Left Distance", getLeftDistance());
        Util.logger(this, telemetry, Level.INFO, "Right Distance", getRightDistance());
        Util.logger(this, telemetry, Level.INFO, "Gyro Angle", getAngle());
    }

    public void tankDrive(double left, double right) {
        drive.tankDrive(left, right);
    }

    public void arcadeDrive(double forward, double rotate) {
        drive.arcadeDrive(forward, rotate);
    }

    public void stop() {
        drive.stop();
    }

    public double getLeftDistance() {
        double currentRevolution = left.getPosition() / (double) DRIVE_TPR;
        return currentRevolution * DRIVE_WHEEL_DIAMETER;
    }

    public double getRightDistance() {
        double currentRevolution = right.getPosition() / (double) DRIVE_TPR;
        return currentRevolution * DRIVE_WHEEL_DIAMETER;
    }

    public double getAngle() {
        return gyro.getHeading();
    }

    public void resetEncoders() {
        left.reset();
        right.reset();
    }

    public void resetAngle() {
        gyro.reset();
    }
}
