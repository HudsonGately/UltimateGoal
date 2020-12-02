package org.firstinspires.ftc.teamcode.commands.wobblegoal;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class MoveWobbleGoal extends CommandBase  {
   private WobbleGoalArm arm;
   private double speed;

   public MoveWobbleGoal(WobbleGoalArm arm, double speed) {
      this.arm = arm;
      this.speed = speed;
      this.addRequirements(this.arm);
   }

   @Override
   public void initialize() {
      arm.setArmSpeed(speed);
   }

   @Override
   public void end(boolean interrupted) {
      arm.setArmSpeed(0);
   }
}
