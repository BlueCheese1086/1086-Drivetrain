package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;

import java.util.Arrays;
import java.util.List;

public class PathFindToNearestPose extends Command {
    private Drivetrain drivetrain;
    private List<Pose2d> poses;

    private Pose2d endPose;
    private int shouldEnds = 0;

    /**
     * Creates a new {@link PathfindToNearestPose} command.
     * It pathfinds to the nearest pose from a list of poses.
     * 
     * @param drivetrain The drivetrain subsystem to control.
     * @param poses The list of poses to select from.
     */
    public PathFindToNearestPose(Drivetrain drivetrain, List<Pose2d> poses) {
        this.drivetrain = drivetrain;
        this.poses = poses;

        addRequirements(drivetrain);
    }

    /**
     * Creates a new {@link PathfindToNearestPose} command.
     * It pathfinds to the nearest pose from a list of poses.
     * 
     * @param drivetrain The drivetrain subsystem to control.
     * @param poses The list of poses to select from.
     */
    public PathFindToNearestPose(Drivetrain drivetrain, Pose2d... poses) {
        this.drivetrain = drivetrain;
        this.poses = Arrays.asList(poses);

        addRequirements(drivetrain);
    }

    /**
     * Called when the command is initially scheduled.
     * 
     * It updates the end pose when the command is initialized to prevent it from changing during pathfinding.
     */
    @Override
    public void initialize() {
        endPose = drivetrain.getPose().nearest(poses);
        shouldEnds = 0;
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * 
     * It gets the state of the trajectory at the current point in time and drives the robot with the output.
     */
    @Override
    public void execute() {
        Pose2d curPose = drivetrain.getPose();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            drivetrain.xController.calculate(curPose.getX(), endPose.getX()),
            drivetrain.yController.calculate(curPose.getY(), endPose.getY()),
            drivetrain.thetaController.calculate(curPose.getRotation().getRadians(), endPose.getRotation().getRadians()),
            curPose.getRotation());

        drivetrain.drive(speeds);
    }

    /**
     * Returns true when the command should end.
     * 
     * It returns true when the current pose is within the set tolerance for 5 ticks in a row (0.1s).
     */
    @Override
    public boolean isFinished() {
        boolean shouldEnd = drivetrain.xController.atSetpoint() && drivetrain.yController.atSetpoint() && drivetrain.thetaController.atSetpoint();

        if (shouldEnd) {
            shouldEnds++;
        } else {
            shouldEnds = 0;
        }

        return shouldEnds > 5;
    }

    /**
     * Called once the command ends or is interrupted.
     * 
     * It sets the speeds of the robot to 0.
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds());
    }
}