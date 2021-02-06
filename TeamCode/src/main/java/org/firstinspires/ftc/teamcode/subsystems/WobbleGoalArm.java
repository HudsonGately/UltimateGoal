package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

public class WobbleGoalArm extends SubsystemBase {
    private Telemetry telemetry;
    private TelemetryPacket packet;
    private CRServo arm;
    private ServoEx claw;
    private boolean clawOpened;

    public WobbleGoalArm(CRServo arm, ServoEx claw, Telemetry tl, TelemetryPacket packet) {
        this.arm = arm;
        this.claw = claw;
        this.clawOpened = true;

        this.telemetry = tl;
        this.packet = packet;
    }

    @Override
    public void periodic() {
        Util.logger(this, telemetry, Level.INFO, "Current Arm Speed", arm.getPower());
        Util.logger(this, telemetry, Level.INFO, "Wobble Claw Pos", claw.getPosition());
    }
    public void setArmSpeed(double speed) {
        arm.setPower(speed);
    }
    public void liftArm() { setArmSpeed(1); }
    public void lowerArm() { setArmSpeed(-1); }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }
    public void openClaw() { setClawPosition(0.78); }
    public void closeClaw() { setClawPosition(0.52); }
    
    public void toggleClaw() {
       clawOpened = !clawOpened; 
       if (clawOpened) {
           openClaw();
       } else {
           closeClaw();
       }
    }
    
    public boolean isClawOpened() {
        return clawOpened;
    }
    public void stopArm() {
        arm.setPower(0);
    }

}
