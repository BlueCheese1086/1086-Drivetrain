package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.Drivetrain;
import java.util.function.Supplier;

public class SwerveDrive extends Command {
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> ySpeedSupplier;
    private Supplier<Double> zSteerSupplier;
    private Supplier<Double> percentSupplier;
    private Supplier<Boolean> fieldRelativeToggleSupplier;
    private Drivetrain drivetrain;

    private boolean fieldRelative = true;

    /**
     * Creates a new SwerveDrive command.
     * It drives the robot based on different percent inputs.
     * 
     * @param drivetrain The drivetrain subsystem to control.
     * @param xSpeedSupplier The supplier for the percent x speed of the robot [-1,1].
     * @param ySpeedSupplier The supplier for the percent y speed of the robot [-1,1].
     * @param zSteerSupplier The supplier for the percent steer speed of the robot [-1,1].
     * @param percentSupplier The supplier for the max percent input. [-1,1]
     * @param toggleFieldRelative The supplier for whether or not to use field relative speeds.
     */
    public SwerveDrive(Drivetrain drivetrain, Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> zSteerSupplier, Supplier<Double> percentSupplier, Supplier<Boolean> toggleFieldRelative) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.zSteerSupplier = zSteerSupplier;
        this.percentSupplier = percentSupplier;
        this.fieldRelativeToggleSupplier = toggleFieldRelative;
        
        addRequirements(drivetrain);
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * 
     * It reads the input and drives the robot accordingly.
     */
    @Override
    public void execute() {
        if (fieldRelativeToggleSupplier.get()) {
            fieldRelative = !fieldRelative;
        }

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
        ChassisSpeeds speeds;
        
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                DriveConstants.maxLinearVelocity.times(-ySpeed * percentSupplier.get()),
                DriveConstants.maxLinearVelocity.times(-xSpeed * percentSupplier.get()),
                DriveConstants.maxAngularVelocity.times(-zSteer * percentSupplier.get()),
                drivetrain.getHeading());
        } else {
            speeds = new ChassisSpeeds(
                DriveConstants.maxLinearVelocity.times(-ySpeed * percentSupplier.get()),
                DriveConstants.maxLinearVelocity.times(-xSpeed * percentSupplier.get()),
                DriveConstants.maxAngularVelocity.times(-zSteer * percentSupplier.get()));
        }

        // Driving the robot
        drivetrain.drive(speeds);
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {}
}