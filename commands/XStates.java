package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

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

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * 
     * Repeatedly sets the robot states to an "X" pattern.
     */
    @Override
    public void execute() {
        drivetrain.xStates();
    }

    /** Returns true when the command should end. */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {}
}
