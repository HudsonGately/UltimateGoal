package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

@Config
public class ShooterFeeder extends SubsystemBase {

    public static double SERVO_POSITION_SHOOT = 0.2;
    public static double SERVO_POSITION_HOME = 0.31;

    private Telemetry telemetry;
    private ServoEx feedServo;

    public ShooterFeeder(ServoEx feedServo, Telemetry tl) {
        this.feedServo = feedServo;
        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        Util.logger(this, telemetry, Level.INFO, "Feed Servo Position", feedServo.getPosition());
    }

    public void setFeedServo(double position) {
        feedServo.setPosition(position);
    }

    public void feedShooter() {
        setFeedServo(SERVO_POSITION_SHOOT);
    }
    public void retractFeed() {
        setFeedServo(SERVO_POSITION_HOME);
    }
}
