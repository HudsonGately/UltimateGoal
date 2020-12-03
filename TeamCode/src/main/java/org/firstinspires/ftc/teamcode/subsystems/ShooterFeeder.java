package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class ShooterFeeder extends SubsystemBase {
    private Telemetry telemetry;
    private ServoEx feedServo;

    public ShooterFeeder(ServoEx feedServo, Telemetry tl) {
        this.feedServo = feedServo;
        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        telemetry.addData("Feed Servo Position", feedServo.getPosition());
    }

    public void setFeedServo(double position) {
        feedServo.setPosition(position);
    }

    public void feedShooter() {
        setFeedServo(Constants.SERVO_POSITION_SHOOT);
    }
    public void retractFeed() {
        setFeedServo(Constants.SERVO_POSITION_HOME);
    }
}
