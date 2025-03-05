package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.Supplier;

public class SwerveDrive extends Command {
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> ySpeedSupplier;
    private Supplier<Double> zSteerSupplier;
    private Drivetrain drivetrain;

    /**
     * Creates a new SwerveDrive command.
     * It drives the robot based on different percent inputs.
     * 
     * @param drivetrain The drivetrain subsystem to control.
     * @param xSpeedSupplier The supplier for the percent x speed of the robot [-1,1].
     * @param ySpeedSupplier The supplier for the percent y speed of the robot [-1,1].
     * @param zSteerSupplier The supplier for the percent steer speed of the robot [-1,1].
     */
    public SwerveDrive(Drivetrain drivetrain, Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> zSteerSupplier) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.zSteerSupplier = zSteerSupplier;
        
        addRequirements(drivetrain);
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * 
     * It reads the input and drives the robot accordingly.
     */
    @Override
    public void execute() {
        // Getting values from the suppliers.
        double xSpeed = xSpeedSupplier.get();
        double ySpeed = ySpeedSupplier.get();
        double zSteer = zSteerSupplier.get();

        // Applying a deadband
        xSpeed = MathUtil.applyDeadband(xSpeed, 0.1);
        ySpeed = MathUtil.applyDeadband(ySpeed, 0.1);
        zSteer = MathUtil.applyDeadband(zSteer, 0.1);

        // When applying a deadband, you cannot go slower than the deadband zone.
        // This code allows the robot to move within that "lost" zone.
        // The problem with this is that it could prevent the robot from moving at its max speed.
        // To prevent this, we stop dampening the speed at 90%.
        if (xSpeed >  0.1 && xSpeed <  0.9) xSpeed -= 0.1;
        if (ySpeed >  0.1 && ySpeed <  0.9) ySpeed -= 0.1;
        if (zSteer >  0.1 && zSteer <  0.9) zSteer -= 0.1;

        if (xSpeed < -0.1 && xSpeed > -0.9) xSpeed += 0.1;
        if (ySpeed < -0.1 && ySpeed > -0.9) ySpeed += 0.1;
        if (zSteer < -0.1 && zSteer > -0.9) zSteer += 0.1;

        // Getting speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            DriveConstants.maxLinearVelocity.times(-ySpeed),
            DriveConstants.maxLinearVelocity.times(-xSpeed),
            DriveConstants.maxAngularVelocity.times(-zSteer),
            drivetrain.getHeading());

        // Driving the robot
        drivetrain.drive(speeds);
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