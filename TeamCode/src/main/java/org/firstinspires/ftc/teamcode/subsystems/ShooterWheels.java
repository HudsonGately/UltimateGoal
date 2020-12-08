package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

/**
 * TODO Think about separating shooter from angler
 */

public class ShooterWheels extends SubsystemBase {
    private Telemetry telemetry;

    private PIDFController shooterWheelsPID;
    private double shooterTarget;
    DcMotorEx frontMotor, backMotor;

    public ShooterWheels(DcMotorEx frontMotor, DcMotorEx backMotor, Telemetry tl) {


        frontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontMotor = frontMotor;
        this.backMotor = backMotor;
        shooterWheelsPID = new PIDFController(Constants.SHOOTER_P, 0, 0, Constants.SHOOTER_F);


        shooterTarget = 0;
        telemetry = tl;
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

        double output = shooterWheelsPID.calculate(getShooterRPM(), shooterTarget);
        setShooterPower(output);

    }

    public void setShooterPower(double speed) {
        frontMotor.setPower(speed);
        backMotor.setPower(speed);
    }
    public void setShooterRPM(double wheelRPM) {
        shooterTarget = wheelRPM;
    }

    public void stopShooter() {
       setShooterPower(0);
    }

    public double getShooterRPM() {
        return 60 * ((double) frontMotor.getVelocity() / (double) Constants.SHOOTER_TPR);
    }


}
