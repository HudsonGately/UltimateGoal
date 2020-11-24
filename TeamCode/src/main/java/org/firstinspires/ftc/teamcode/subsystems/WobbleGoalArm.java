package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

public class WobbleGoalArm extends SubsystemBase {
    private CRServo arm;
    private ServoEx claw;

    WobbleGoalArm(CRServo arm, ServoEx claw) {
        this.arm = arm;
        this.claw = claw;
    }

    public void setArmSpeed(double speed) {
        arm.set(speed);
    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }

    public void stopArm() {
        arm.set(0);
    }

}
