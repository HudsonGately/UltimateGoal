package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

public class WobbleGoalArm extends SubsystemBase {
    private Telemetry telemetry;
    private CRServo arm;
    private ServoEx claw;
    private boolean clawOpened;

    public WobbleGoalArm(CRServo arm, ServoEx claw, Telemetry tl) {
        this.arm = arm;
        this.claw = claw;
        this.clawOpened = true;

        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        Util.logger(this, telemetry, Level.INFO, "Current Arm Speed", arm.get());
        Util.logger(this, telemetry, Level.INFO, "Wobble Claw Pos", claw.getPosition());
    }

    public void setArmSpeed(double speed) {
        arm.set(speed);
    }
    public void liftArm() { setArmSpeed(1); }
    public void lowerArm() { setArmSpeed(-1); }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }
    public void openClaw() { setClawPosition(0.5); }
    public void closeClaw() { setClawPosition(0); }
    
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
        arm.set(0);
    }

}
