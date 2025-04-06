package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;

public class XStates extends Command {
    private Drivetrain drivetrain;

    /**
     * Creates a new XStates command.
     * It sets the states of the robot to an "X" pattern to resist movement.
     */
    public XStates(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * 
     * Repeatedly sets the robot states to an "X" pattern.
     */
    @Override
    public void execute() {
        drivetrain.xStates();
    }
}