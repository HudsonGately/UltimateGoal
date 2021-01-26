package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

public class WobbleGoalArm extends SubsystemBase {
    private Telemetry telemetry;
    private TelemetryPacket packet;
    private MotorEx arm;
    private ServoEx claw, lazySusan;
    private boolean automatic;
    private double targetDegree;

    public static double CPR = 2786;
    public static double ARM_SPEED = 0.6;

    public WobbleGoalArm(MotorEx arm, ServoEx claw, ServoEx lazySusan, Telemetry tl, TelemetryPacket packet) {
        this.arm = arm;
        this.arm.setDistancePerPulse(360/CPR);
        this.targetDegree = 0;
        this.arm.setTargetDistance(targetDegree);
        this.arm.setPositionTolerance(20);

        this.claw = claw;
        this.lazySusan = lazySusan;
        this.telemetry = tl;
        this.packet = packet;
        automatic = true;
    }

    public void toggleAutomatic() {
        automatic = !automatic;
    }

    public boolean isAutomatic() {
        return automatic;
    }

    @Override
    public void periodic() {
        if (automatic) {
            arm.setRunMode(Motor.RunMode.PositionControl);
            arm.setTargetDistance(targetDegree);
            arm.set(ARM_SPEED);
        } else {
            arm.setRunMode(Motor.RunMode.RawPower);
        }

        Util.logger(this, telemetry, Level.INFO, "Wobble Claw Pos", claw.getPosition());
    }



    public void liftArm() {
        if (automatic)
            arm.set(-ARM_SPEED);
    }
    public void lowerArm() {
        if (automatic)
            arm.set(ARM_SPEED);
    }
    public void stopArm() {
        arm.setRunMode(Motor.RunMode.RawPower);
        arm.set(0);
        arm.stopMotor();
    }
    public void placeWobbleGoal() { targetDegree = 30; }
    public void liftWobbleGoal() { targetDegree = 0; }
    public boolean atTargetAngle() {
        return arm.atTargetPosition();
    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }
    public void openClaw() { setClawPosition(0.74); }
    public void closeClaw() { setClawPosition(0.52); }

    public void setLazySusanPosition(double position) {
        lazySusan.setPosition(position);
    }
}
