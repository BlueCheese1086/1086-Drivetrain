package frc.robot.subsystems.drivetrain.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.ArrayList;

public class PathfindToPose extends Command {
    private Drivetrain drivetrain;
    private Pose2d endPose;

    private Trajectory trajectory;
    private Timer timer;

    private HolonomicDriveController holonomicController;

    /**
     * Creates a new PathfindToPose command.
     * It controls the drivetrain and moves it to a certain pose on the field, while also travelling to other poses on the way.
     * 
     * @param drivetrain The drivetrain subsystem to control.
     * @param endPose The pose to pathfind to.
     */
    public PathfindToPose(Drivetrain drivetrain, Pose2d endPose) {
        this.drivetrain = drivetrain;
        this.endPose = endPose;

        timer = new Timer();
        timer.start();

        addRequirements(drivetrain);
    }

    public void setGoalPose(Pose2d goalPose) {
        this.endPose = goalPose;
    }

    /**
     * Called when the command is initially scheduled.
     * 
     * It generates the trajectory for the robot to follow.
     */
    @Override
    public void initialize() {
        timer.reset();

	    holonomicController = drivetrain.getController();

        // Setting the tolerance of the holonomic controller
        // For some reason this is a pose2d?
        // Basically if the error is within these bounds, the controller says it is done with that part of the path.
        holonomicController.setTolerance(new Pose2d(0.1, 0.1, new Rotation2d(0.02)));

        // Initializing the trajectory config
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DriveConstants.maxLinearVelocity, DriveConstants.maxLinearAcceleration);
        trajectoryConfig.setEndVelocity(MetersPerSecond.zero());
        trajectoryConfig.setKinematics(drivetrain.getKinematics());

        // Generating the trajectory to follow;
        ArrayList<Pose2d> internalPoses = new ArrayList<Pose2d>();
        
        // The WPILib trajectory generator needs at least two poses, so this is adding the two
        internalPoses.add(drivetrain.getPose());
        internalPoses.add(endPose);

        trajectory = TrajectoryGenerator.generateTrajectory(internalPoses, trajectoryConfig);
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * 
     * It gets the state of the trajectory at the current point in time and drives the robot with the output.
     */
    @Override
    public void execute() {
        // Sampling the state for this current point in time
        Trajectory.State state = trajectory.sample(timer.get());

        // Driving the robot based on the holonomic controller output
        drivetrain.drive(holonomicController.calculate(drivetrain.getPose(), state, state.poseMeters.getRotation()));
    }

    /**
     * Returns true when the command should end.
     * 
     * It returns true when the trajectory is within the tolerance of the last pose.
     * The check of if its the last pose is necessary, otherwise it'd return true immediately as it would have reached any position within the trajectory.
     */
    @Override
    public boolean isFinished() {
        return trajectory.sample(timer.get()).poseMeters.equals(endPose) && holonomicController.atReference();
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
