package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Constants;

public class Intake extends SubsystemBase {
    private MotorEx intake;
    private ServoEx feedServo; 

    public Intake(MotorEx intake, ServoEx feedServo) {
        this.feedServo = feedServo;
        this.intake = intake;
    }

    private void set(double speed) {
        intake.set(speed);
    }

    public void intake() {
        set(Constants.INTAKE_SPEED);
    }

    public void setFeedServo(double position) {
        feedServo.setPosition(position);
    }
    public void outtake() {
        set(Constants.OUTAKE_SPEED);
    }

    public void stop() {
        intake.stopMotor();
    }
}
