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
public class ShooterWheels extends SubsystemBase {
    private MotorGroup shooterMotors;
    private Motor.Encoder shooterEncoder, anglerEncoder;
    private Motor anglerMotor;

    public ShooterWheels(MotorEx frontMotor, MotorEx backMotor) {
        shooterMotors = new MotorGroup(frontMotor, backMotor);
        shooterEncoder = backMotor.encoder;

    }

    public void setShooter(double speed) {
        shooterMotors.set(speed);
    }


    public void stopShooter() {
        shooterMotors.stopMotor();
    }

    public double getShooterRPM() {
        return 60 * ((double) shooterEncoder.getCorrectedVelocity() / (double) Constants.SHOOTER_TPR);
    }




}
