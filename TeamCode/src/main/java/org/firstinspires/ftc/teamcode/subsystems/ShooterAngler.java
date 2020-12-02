package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Constants;

public class ShooterAngler extends SubsystemBase {
    private MotorEx anglerMotor;
    private Motor.Encoder anglerEncoder;

    public ShooterAngler(MotorEx anglerMotor) {

        this.anglerMotor = anglerMotor;
        this.anglerEncoder = anglerMotor.encoder;
        this.anglerEncoder.reset();
    }
    public void setAngler(double speed) {
        anglerMotor.set(speed);
    }
    public void stopAngler() {
        anglerMotor.stopMotor();
    }

    public double getShooterAngle() {
        double revolutions = anglerEncoder.getPosition() / (double) Constants.ANGLER_TPR;
        return revolutions * 360 + Constants.SHOOTER_OFFSET_ANGLE;
    }
}
