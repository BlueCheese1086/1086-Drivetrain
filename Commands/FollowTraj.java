// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.Commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AdjustableValues;
import frc.robot.util.PoseAllignment;

import java.util.List;
import org.littletonrobotics.junction.Logger;

//FYI pid vals arent done right change later
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowTraj extends Command {

    private PIDController autoXPID = new PIDController(
            AdjustableValues.getNumber("LineXKP"),
            AdjustableValues.getNumber("LineXKI"),
            AdjustableValues.getNumber("LineXKD"));

    private PIDController autoYPID = new PIDController(
            AdjustableValues.getNumber("LineYKP"),
            AdjustableValues.getNumber("LineYKI"),
            AdjustableValues.getNumber("LineYPKD"));

    private ProfiledPIDController trajHeading = new ProfiledPIDController(
            AdjustableValues.getNumber("LineRotKP"),
            AdjustableValues.getNumber("LineRotKI"),
            AdjustableValues.getNumber("LineRotKD"),
            new Constraints(4.30, 3.99));

    public TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            LinearVelocity.ofBaseUnits(4.30, MetersPerSecond),
            LinearAcceleration.ofBaseUnits(3.99, MetersPerSecondPerSecond))
            .setReversed(false)
            .setStartVelocity(0.0)
            .setEndVelocity(0.0);

    public HolonomicDriveController trajDriveController = new HolonomicDriveController(autoXPID, autoYPID, trajHeading);
    public PoseAllignment poseAllignment = new PoseAllignment();
    public Drive drive;
    private Timer timer = new Timer();
    private double pathDuro = 0.0;
    private Pose2d robot;
    private Pose2d target = null;
    private Trajectory traj;

    public FollowTraj(Drive drive, Pose2d robot) {
        this.drive = drive;
        this.robot = robot;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.target = drive.autoLeftPose(Constants.allianceMode);
        Logger.recordOutput("PID/Align", target);
        traj = TrajectoryGenerator.generateTrajectory(robot, List.of(), target, trajectoryConfig);

        pathDuro = traj.getTotalTimeSeconds();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // double time = timer.get();
        Trajectory.State desiredState = traj.sample(traj.getTotalTimeSeconds());

        ChassisSpeeds zoom = trajDriveController.calculate(robot, desiredState, target.getRotation());
        drive.runVelocity(zoom);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());
        // timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return timer.hasElapsed(pathDuro);
        return false;
    }
}
