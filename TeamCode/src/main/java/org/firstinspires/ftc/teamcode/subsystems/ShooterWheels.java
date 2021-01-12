package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

/**
 * TODO Think about separating shooter from angler
 */
@Config
public class ShooterWheels extends SubsystemBase {
    private Telemetry telemetry;

    public static double MAX_SHOOTER_RPM = 3886;
    public static int SHOOTER_WHEEL_DIAMETER = 4;
    public static double SHOOTER_TPR = 28;

    public static double SHOOTER_P = 0.002;
    public static double SHOOTER_F = 1.0 / MAX_SHOOTER_RPM;


    public static double TARGET_SPEED = 3000;
    private PIDFController shooterWheelsPID;
    private double shooterTarget, offset;
    DcMotorEx frontMotor, backMotor;

    public ShooterWheels(DcMotorEx frontMotor, DcMotorEx backMotor, Telemetry tl) {


        frontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontMotor = frontMotor;
        this.backMotor = backMotor;
        shooterWheelsPID = new PIDFController(SHOOTER_P, 0, 0, SHOOTER_F);

        shooterWheelsPID.setTolerance(50);

        shooterTarget = 0;
        telemetry = tl;
        offset = 0;
    }

    @Override
    public void periodic() {
        handleShooterPID();
        Util.logger(this, telemetry, Level.INFO, "Shooter RPM", getShooterRPM());
        Util.logger(this, telemetry, Level.INFO, "Shooter Setpoint", shooterWheelsPID.getSetPoint());
        Util.logger(this, telemetry, Level.INFO, "Shooter Power", frontMotor.getPower());
    }

    private void handleShooterPID() {
        if (shooterTarget == 0) {
            setShooterPower(0);
            return;
        }

        double output = shooterWheelsPID.calculate(getShooterRPM(), shooterTarget + offset);
        setShooterPower(output);

    }

    public void setShooterPower(double speed) {
        frontMotor.setPower(speed);
        backMotor.setPower(speed);
    }
    public void setShooterRPM(double wheelRPM) {
        shooterTarget = wheelRPM;
    }
    public void adjustShooterRPM(double adjustment) {
        offset += adjustment;
    }

    public void stopShooter() {
       setShooterPower(0);
    }

    public double getShooterRPM() {
        return (60 * ((double) frontMotor.getVelocity() / (double) SHOOTER_TPR)) / 1.125;
    }

    public boolean atSetpoint() {
        return shooterWheelsPID.atSetPoint();
    }


}
