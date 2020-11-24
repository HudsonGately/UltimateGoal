package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * TODO Think about separating shooter from angler
 */
public class Shooter extends SubsystemBase {
    private MotorGroup shooterMotors;
    private Motor.Encoder shooterEncoder, anglerEncoder;
    private Motor anglerMotor;
    private ServoEx feedServo;

    Shooter(MotorEx frontMotor, MotorEx backMotor, MotorEx anglerMotor, ServoEx feedServo) {
        shooterMotors = new MotorGroup(frontMotor, backMotor);
        shooterEncoder = backMotor.encoder;

        this.anglerMotor = anglerMotor;
        this.anglerEncoder = anglerMotor.encoder;
        this.anglerEncoder.reset();
        this.feedServo = feedServo;
    }

    public void setShooter(double speed) {
        shooterMotors.set(speed);
    }

    public void setAngler(double speed) {
        anglerMotor.set(speed);
    }

    public void stopShooter() {
        shooterMotors.stopMotor();
    }

    public void stopAngler() {
        anglerMotor.stopMotor();
    }

    public void setFeedServo(double position) {
        feedServo.setPosition(position);
    }

    public double getShooterRPM() {
        return 60 * ((double) shooterEncoder.getCorrectedVelocity() / (double) Constants.SHOOTER_TPR);
    }

    public double getShooterAngle() {
        double revolutions = anglerEncoder.getPosition() / (double) Constants.ANGLER_TPR;
        return revolutions * 360 + Constants.SHOOTER_OFFSET_ANGLE;
    }



}
