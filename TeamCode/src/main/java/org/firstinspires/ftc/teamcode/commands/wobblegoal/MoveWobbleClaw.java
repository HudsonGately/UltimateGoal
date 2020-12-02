package org.firstinspires.ftc.teamcode.commands.wobblegoal;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class MoveWobbleClaw extends CommandBase  {
   private WobbleGoalArm arm;
   private double position;

   public MoveWobbleClaw(WobbleGoalArm arm, double position) {
      this.arm = arm;
      this.position = position;
      this.addRequirements(this.arm);
   }

   @Override
   public void initialize() {
      arm.setClawPosition(position);
   }


   @Override
   public boolean isFinished() {
      return true;
   }
}
