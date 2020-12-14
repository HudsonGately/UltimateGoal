package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;
import java.util.logging.Logger;
@Config
public class ShooterAngler extends SubsystemBase {

    public static double SHOOTER_OFFSET_ANGLE = 30.4;
    public static double ANGLER_TPR = 5264;
    public static double ANGLER_P = 0;
    public static double ANGLER_D = 0;

    Telemetry telemetry;

    private MotorEx anglerMotor;
    private Motor.Encoder anglerEncoder;
    private PIDController pidController;
    private boolean debug;
    private double angleTarget;

    private static final Logger LOGGER = Logger.getLogger( ShooterAngler.class.getName() );
    public ShooterAngler(MotorEx anglerMotor, Telemetry tl, boolean debug) {

        this.anglerMotor = anglerMotor;
        this.anglerMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.anglerEncoder = anglerMotor.encoder;
        this.anglerEncoder.reset();

        this.pidController = new PIDController(ANGLER_P, 0, ANGLER_D);
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

        Util.logger(this, telemetry, Level.INFO, "Current Angler Angle", getShooterAngle());
        Util.logger(this, telemetry, Level.INFO, "Current Angler Power", anglerMotor.get());
    }

    private void handlePID() {

        double output = pidController.calculate(getShooterAngle(), angleTarget);
    }


    public void setShooterAngle(double angle) {
        angleTarget = angle;
    }

    public double getShooterAngle() {
        double revolutions = anglerEncoder.getPosition() / (double) ANGLER_TPR;
        return SHOOTER_OFFSET_ANGLE - revolutions * 360;
    }
}
