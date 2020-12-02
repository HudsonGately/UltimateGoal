package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

public class WobbleGoalArm extends SubsystemBase {
    private CRServo arm;
    private ServoEx claw;
    private boolean clawOpened;
    public WobbleGoalArm(CRServo arm, ServoEx claw) {
        this.arm = arm;
        this.claw = claw;
        this.clawOpened = true;
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
