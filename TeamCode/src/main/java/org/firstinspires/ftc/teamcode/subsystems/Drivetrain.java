package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.logging.Level;
import java.util.logging.Logger;

public class Drivetrain extends SubsystemBase {
    Telemetry telemetry;

    private DifferentialDrive drive;
    private GyroEx gyro;
    private Motor.Encoder left, right;
    private static final Logger LOGGER = Logger.getLogger( Drivetrain.class.getName() );
    public Drivetrain(MotorEx backLeft, MotorEx backRight, MotorEx frontLeft, MotorEx frontRight, GyroEx gyro, Motor.Encoder leftEncoder, Motor.Encoder rightEncoder, Telemetry tl) {
        this.gyro = gyro;
        drive = new DifferentialDrive(false, new MotorGroup(backLeft, frontLeft), new MotorGroup(backRight, frontRight));
        this.left = leftEncoder;
        this.right = rightEncoder;
        this.telemetry = tl;
        LOGGER.setLevel(Level.ALL);
    }

    public Drivetrain(MotorEx backLeft, MotorEx backRight, MotorEx frontLeft, MotorEx frontRight, GyroEx gyro, Telemetry tl) {
        this(backLeft, backRight, frontLeft, frontRight, gyro, backLeft.encoder, backRight.encoder, tl);
    }

    @Override
    public void periodic() {
        LOGGER.log(Level.INFO, "Current Left Distance: {-999}", getLeftDistance());
        LOGGER.log(Level.INFO, "Current Right Distance: {-999}", getRightDistance());
        LOGGER.log(Level.INFO, "Current Gyro Angle: {0}", getAngle());
        // Might need to get rid of this for performance issues
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
