package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Constants;

public class ShooterFeeder extends SubsystemBase {
    private ServoEx feedServo;

    public ShooterFeeder(ServoEx feedServo) {
        this.feedServo = feedServo;
    }

    public void setFeedServo(double position) {
        feedServo.setPosition(position);
    }
}
