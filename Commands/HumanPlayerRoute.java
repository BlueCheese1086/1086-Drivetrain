// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.ControlConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HumanPlayerRoute extends Command {
    Drive drive;
    private Command autoBuilder;
    private PathConstraints constraints = new PathConstraints(
            ControlConstants.maxVelocity,
            ControlConstants.maxAccel,
            ControlConstants.angleMaxVelocity,
            ControlConstants.angleMaxAccel);
    private boolean alliance;

    public HumanPlayerRoute(Drive drive, boolean alliance) {
        this.drive = drive;
        this.alliance = alliance;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.autoBuilder = AutoBuilder.pathfindToPose(drive.autoHP(alliance), constraints, 2.0);
        autoBuilder.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            autoBuilder.cancel();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return autoBuilder.isFinished();
    }
}
