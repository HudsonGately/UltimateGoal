package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * TODO Think about separating shooter from angler
 */

public class ShooterWheels extends SubsystemBase {
    private Telemetry telemetry;

    private MotorGroup shooterMotors;
    private Motor.Encoder shooterEncoder, anglerEncoder;
    private PIDController shooterWheelsPID;
    private double shooterTarget;

    public ShooterWheels(MotorEx frontMotor, MotorEx backMotor, Telemetry tl) {
        shooterMotors = new MotorGroup(frontMotor, backMotor);
        shooterEncoder = backMotor.encoder;
        shooterWheelsPID = new PIDController(Constants.SHOOTER_P, 0, 0);

        shooterTarget = 0;
        telemetry = tl;
    }

    @Override
    public void periodic() {
        handleShooterPID();
        telemetry.addData("Shooter RPM", getShooterRPM());
        telemetry.addData("Shooter Power", shooterMotors.get());
    }

    private void handleShooterPID() {
        if (shooterTarget == 0) {
            setShooterPower(0);
            return;
        }

        double output = shooterWheelsPID.calculate(getShooterRPM(), shooterTarget) + shooterTarget * Constants.SHOOTER_F;
        setShooterPower(output);

    }

    public void setShooterPower(double speed) {
        shooterMotors.set(speed);
    }
    public void setShooterRPM(double wheelRPM) {
        shooterTarget = wheelRPM;
    }

    public void stopShooter() {
        shooterMotors.stopMotor();
    }

    public double getShooterRPM() {
        return 60 * ((double) shooterEncoder.getCorrectedVelocity() / (double) Constants.SHOOTER_TPR);
    }


}
