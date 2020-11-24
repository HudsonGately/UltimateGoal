package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.Constants;

public class Drivetrain extends SubsystemBase {
    private DifferentialDrive drive;
    private GyroEx gyro;
    private Motor.Encoder left, right;

    Drivetrain(MotorEx backLeft, MotorEx backRight, MotorEx frontLeft, MotorEx frontRight, GyroEx gyro, Motor.Encoder leftEncoder, Motor.Encoder rightEncoder) {
        this.gyro = gyro;
        drive = new DifferentialDrive(new MotorGroup(backLeft, frontLeft), new MotorGroup(backRight, frontRight));
        this.left = leftEncoder;
        this.right = rightEncoder;
    }

    public Drivetrain(MotorEx backLeft, MotorEx backRight, MotorEx frontLeft, MotorEx frontRight, GyroEx gyro) {
        this(backLeft, backRight, frontLeft, frontRight, gyro, backLeft.encoder, backRight.encoder);
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
        double currentRevolution = left.getPosition() / (double) Constants.DRIVE_TPR;
        return currentRevolution * Constants.DRIVE_WHEEL_DIAMETER;
    }

    public double getRightDistance() {
        double currentRevolution = right.getPosition() / (double) Constants.DRIVE_TPR;
        return currentRevolution * Constants.DRIVE_WHEEL_DIAMETER;
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
