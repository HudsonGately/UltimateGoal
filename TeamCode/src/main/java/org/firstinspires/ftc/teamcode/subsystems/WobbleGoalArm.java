package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

@Config
public class WobbleGoalArm extends SubsystemBase {
    private Telemetry telemetry;
    private TelemetryPacket packet;
    private MotorEx arm;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0, 0, 0, 0.8);
    public static double OFFSET = 0;
    private PIDFController controller;
    private ServoEx claw, lazySusan;
    private boolean automatic;

    public static double CPR = 2786;
    public static double ARM_SPEED = 0.6;

    public WobbleGoalArm(MotorEx arm, ServoEx claw, ServoEx lazySusan, Telemetry tl, TelemetryPacket packet) {
        this.arm = arm;
        this.arm.setDistancePerPulse(360/CPR);

        controller = new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f, 0, 0);
        controller.setTolerance(10);

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
           controller.setF(pidfCoefficients.f * Math.cos(Math.toRadians(controller.getSetPoint())));
           double output = controller.calculate(getAngle());
           arm.set(output);
        }

        Util.logger(this, telemetry, Level.INFO, "Wobble Claw Pos", claw.getPosition(), packet);
        Util.logger(this, telemetry, Level.INFO, "Wobble Angle", getAngle(), packet);
        Util.logger(this, telemetry, Level.INFO, "Wobble Goal", controller.getSetPoint(), packet);
        Util.logger(this, telemetry, Level.INFO, "Wobble Power", arm.get(), packet);
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
        arm.stopMotor();
        controller.setSetPoint(getAngle());
        automatic = false;
    }
    public void resetEncoder() {
        arm.resetEncoder();
    }

    public double getAngle() {
        return OFFSET + arm.getDistance();
    }

    public void placeWobbleGoal() {
        automatic = true;
        controller.setSetPoint(0);
    }
    public void liftWobbleGoal() {
        automatic = true;
        controller.setSetPoint(30);
    }
    public boolean atTargetAngle() {
        return controller.atSetPoint();
    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }
    public void openClaw() { setClawPosition(0.82); }
    public void closeClaw() { setClawPosition(0.52); }

    public void setLazySusanPosition(double position) {
        lazySusan.setPosition(position);
    }
}
