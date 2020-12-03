package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Intake extends SubsystemBase {
    Telemetry telemetry;
    private MotorEx intake;

    public Intake(MotorEx intake, Telemetry tl) {
        this.intake = intake;
        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        telemetry.addData("Intake Speed", intake.get());
    }

    private void set(double speed) {
        intake.set(speed);
    }

    public void intake() {
        set(Constants.INTAKE_SPEED);
    }

    public void outtake() {
        set(Constants.OUTAKE_SPEED);
    }

    public void stop() {
        intake.stopMotor();
    }
}
