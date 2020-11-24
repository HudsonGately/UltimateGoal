package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private Drivetrain drive;
    private DoubleSupplier leftX, leftY, rightX, rightY;

    protected double multiplier;

    public DefaultDriveCommand(Drivetrain drive, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier rightY) {
        this.drive = drive;
        this.leftX = leftX;
        this.rightX = rightX;
        this.leftY = leftY;
        this.rightY = rightY;

        this.multiplier = 1;
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        // Arcade Drive
        // drive.arcadeDrive(leftY.getAsDouble() * multiplier, rightX.getAsDouble() * multiplier);

        // Tank Drive
        drive.tankDrive(leftY.getAsDouble() * multiplier, rightY.getAsDouble() * multiplier);
    }

}
