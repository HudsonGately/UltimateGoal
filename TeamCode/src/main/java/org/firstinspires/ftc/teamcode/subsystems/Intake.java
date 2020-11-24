package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Constants;

public class Intake extends SubsystemBase {
    private MotorEx intake;

    public Intake(MotorEx intake) {
        this.intake = intake;
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
