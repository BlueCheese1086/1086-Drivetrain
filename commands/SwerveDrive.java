package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.Supplier;

public class SwerveDrive extends Command {
    private Supplier<Double> leftXSupplier;
    private Supplier<Double> leftYSupplier;
    private Supplier<Double> rightXSupplier;
    private Drivetrain drivetrain;

    public SwerveDrive(Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier, Supplier<Double> rightXSupplier) {
        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;
        this.rightXSupplier = rightXSupplier;

        drivetrain = Drivetrain.getInstance();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Getting values from the suppliers.
        double leftXSpeed = leftXSupplier.get();
        double leftYSpeed = leftYSupplier.get();
        double rightXSpeed = rightXSupplier.get();

        // Applying a deadband
        leftXSpeed = MathUtil.applyDeadband(leftXSpeed, 0.1);
        leftYSpeed = MathUtil.applyDeadband(leftYSpeed, 0.1);
        rightXSpeed = MathUtil.applyDeadband(rightXSpeed, 0.1);

        // When applying a deadband, you cannot go slower than the deadband zone.
        // This code allows the robot to move within that "lost" zone.
        // The problem with this is that it could prevent the robot from moving at its max speed.
        // To prevent this, we stop dampening the speed at 90%.
        if ( leftXSpeed >  0.1 &&  leftXSpeed <  0.9)  leftXSpeed -= 0.1;
        if ( leftYSpeed >  0.1 &&  leftYSpeed <  0.9)  leftYSpeed -= 0.1;
        if (rightXSpeed >  0.1 && rightXSpeed <  0.9) rightXSpeed -= 0.1;

        if ( leftXSpeed < -0.1 &&  leftXSpeed > -0.9)  leftXSpeed += 0.1;
        if ( leftYSpeed < -0.1 &&  leftYSpeed > -0.9)  leftYSpeed += 0.1;
        if (rightXSpeed < -0.1 && rightXSpeed > -0.9) rightXSpeed += 0.1;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            DriveConstants.maxLinearVelocity.times(-leftYSpeed),
            DriveConstants.maxLinearVelocity.times(-leftXSpeed),
            DriveConstants.maxAngularVelocity.times(-rightXSpeed),
            drivetrain.getHeading());
            
        // Driving the robot
        drivetrain.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}