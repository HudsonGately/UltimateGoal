package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.logging.Logger;

public class ShooterAngler extends SubsystemBase {
    Telemetry telemetry;

    private MotorEx anglerMotor;
    private Motor.Encoder anglerEncoder;
    private PIDController pidController;
    private boolean debug;
    private double angleTarget;

    private static final Logger LOGGER = Logger.getLogger( ShooterAngler.class.getName() );
    public ShooterAngler(MotorEx anglerMotor, Telemetry tl, boolean debug) {

        this.anglerMotor = anglerMotor;
        this.anglerEncoder = anglerMotor.encoder;
        this.anglerEncoder.reset();

        this.pidController = new PIDController(Constants.ANGLER_P, 0, Constants.ANGLER_D);
        this.pidController.setTolerance(2);

        this.angleTarget = 0;
        this.pidController.setSetPoint(0);
        this.debug = debug;
        telemetry = tl;
    }

    public ShooterAngler(MotorEx anglerMotor, Telemetry tl) {
        this(anglerMotor, tl, false);
    }
    public void setAngler(double speed) {
        anglerMotor.set(speed);
    }
    public void stopAngler() {
        anglerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (debug) {
            handlePID();
        }

        telemetry.addData("Current Shooter Angle", getShooterAngle());
        telemetry.addData("Angler Power", anglerMotor.get());
        LOGGER.info("Current shooter angle: "  + getShooterAngle());
    }

    private void handlePID() {

        double output = pidController.calculate(getShooterAngle(), angleTarget) + Constants.GRAV_FF * Math.cos(Math.toRadians(angleTarget));
    }


    public void setShooterAngle(double angle) {
        angleTarget = angle;
    }

    public double getShooterAngle() {
        double revolutions = anglerEncoder.getPosition() / (double) Constants.ANGLER_TPR;
        return Constants.SHOOTER_OFFSET_ANGLE - revolutions * 360;
    }
}
