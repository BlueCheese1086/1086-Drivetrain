package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Poses;
import frc.robot.subsystems.drivetrain.Drivetrain;


import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class PathFindToLeft extends Command {
    private Drivetrain drivetrain;
    private Pose2d endPose;
    private Pose2d goalPose;
    private Pose2d robotPose;
    private Supplier<Boolean> override;
    private List<Pose2d> leftPoses = new ArrayList<>();

    private int shouldEnds = 0;

    /**
     * Creates a new PathfindToPose command.
     * It controls the drivetrain and moves it to a certain pose on the field, while also travelling to other poses on the way.
     * 
     * @param drivetrain The drivetrain subsystem to control.
     * @param endPose The pose to pathfind to.
     * @param endEarly A boolean supplier that allows the command to be overriden.
     */
    public PathFindToLeft(Drivetrain drivetrain, Supplier<Boolean> endEarly) {
        this.drivetrain = drivetrain;
        this.override = endEarly;
        this.robotPose = drivetrain.getPose();

        leftPoses.add(Poses.REEF_Side1Left);
        leftPoses.add(Poses.REEF_Side2Left);
        leftPoses.add(Poses.REEF_Side3Left);
        leftPoses.add(Poses.REEF_Side4Left);
        leftPoses.add(Poses.REEF_Side5Left);
        leftPoses.add(Poses.REEF_Side6Left);


        addRequirements(drivetrain);
    }

    /**
     * Called when the command is initially scheduled.
     * 
     * It updates the end pose when the command is initialized to prevent the goalPose from changing during pathfinding
     */
    @Override
    public void initialize() {
        endPose = robotPose.nearest(leftPoses);
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
            drivetrain.xController.calculate(curPose.getX(), endPose.getX()),// * (drivetrain.thetaController.atSetpoint() ? 1 : AdjustableValues.getNumber("PID_Dampen")),
            drivetrain.yController.calculate(curPose.getY(), endPose.getY()),// * (drivetrain.thetaController.atSetpoint() ? 1 : AdjustableValues.getNumber("PID_Dampen")),
            drivetrain.thetaController.calculate(curPose.getRotation().getRadians(), endPose.getRotation().getRadians()),
            curPose.getRotation());

        drivetrain.drive(speeds);
    }

    /**
     * Returns true when the command should end.
     * 
     * It returns true when the current pose is within the set tolerance for 5 ticks in a row (0.1s).
     * It will also return true if the override is active.
     */
    @Override
    public boolean isFinished() {
        if (override.get()) return true;

        boolean shouldEnd = drivetrain.xController.atSetpoint() && drivetrain.yController.atSetpoint() && drivetrain.thetaController.atSetpoint();

        System.out.println(shouldEnd);
        if (shouldEnd) shouldEnds++;
        else shouldEnds = 0;

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
