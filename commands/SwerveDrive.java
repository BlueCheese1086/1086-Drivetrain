
package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TurboLogger;
import frc.robot.util.MathUtils;
import java.util.function.Supplier;

public class SwerveDrive extends Command {
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> ySpeedSupplier;
    private Supplier<Double> zSteerSupplier;
    private Supplier<Boolean> fieldRelativeToggleSupplier;
    private Drive drivetrain;

    // Defaulting to field relative control
    private boolean fieldRelative = true;
    private boolean lastToggleVal = false;

    /**
     * Creates a new {@link SwerveDrive} command.
     * It drives the robot based on different percent inputs.
     *
     * @param drivetrain The {@link Drive} subsystem to control.
     * @param xSpeedSupplier The double supplier for the percent x speed of the robot [-1,1].
     * @param ySpeedSupplier The double supplier for the percent y speed of the robot [-1,1].
     * @param zSteerSupplier The double supplier for the percent steer speed of the robot [-1,1].
     * @param percentSupplier The double supplier for the max percent input. [0,1]
     * @param toggleFieldRelative The boolean supplier for whether or not to use field relative speeds.
     */
    public SwerveDrive(Drive drivetrain, Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> zSteerSupplier, Supplier<Boolean> toggleFieldRelative) {
        this.drivetrain = drivetrain;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.zSteerSupplier = zSteerSupplier;
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
        // Only toggling field relative drive on the first tick that the toggle condition is true.
        // Otherwise it would constantly switch between the two modes, confusing the robot.
        boolean toggleVal = fieldRelativeToggleSupplier.get();
        if (toggleVal && !lastToggleVal) {
            fieldRelative = !fieldRelative;
        }

        lastToggleVal = toggleVal;

        // Getting values from the suppliers.
        // Applying a deadband with an offset and then letting the normal values come through at an input of 0.9.
        double xSpeed = MathUtils.applyDeadbandWithOffsets(xSpeedSupplier.get(), 0.1, 0.9);
        double ySpeed = MathUtils.applyDeadbandWithOffsets(ySpeedSupplier.get(), 0.1, 0.9);
        double zSteer = MathUtils.applyDeadbandWithOffsets(zSteerSupplier.get(), 0.1, 0.9);

        // Applying max speeds
        xSpeed *= TurboLogger.get("DriveX_Percent", DriveConstants.driveXPercent);
        ySpeed *= TurboLogger.get("DriveY_Percent", DriveConstants.driveYPercent);
        zSteer *= TurboLogger.get("Steer_Percent", DriveConstants.steerPercent);

        // Getting speeds
        ChassisSpeeds speeds = new ChassisSpeeds(
            DriveConstants.maxLinearVelocity.times(-xSpeed),
            DriveConstants.maxLinearVelocity.times(-ySpeed),
            DriveConstants.maxAngularVelocity.times(-zSteer));

        // Checking whether to drive field relative or not.
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drivetrain.getPose().getRotation());
        }

        // Driving the robot
        drivetrain.drive(speeds);
    }
}
