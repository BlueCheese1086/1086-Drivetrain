package frc.robot.subsystems.drive.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.Characterization;
import frc.robot.subsystems.drive.DriveConstants.ControlConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
    private static final PIDController xPID = new PIDController(AutoConstants.Gains.x.kP, 0.0, 0.0);
    private static final PIDController yPID = new PIDController(AutoConstants.Gains.y.kP, 0.0, 0.0);
    private static final PIDController headingPID = new PIDController(AutoConstants.Gains.heading.kP, 0.0, 0.0);

    private DriveCommands() {
        xPID.setTolerance(0.03);
        yPID.setTolerance(0.03);
        headingPID.setTolerance(0.03);
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y, double deadband) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and
     * angular velocities).
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            DoubleSupplier deadbandSupplier,
            DoubleSupplier percentSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                            Math.signum(xSupplier.getAsDouble()) * Math.pow(xSupplier.getAsDouble(), 2),
                            Math.signum(ySupplier.getAsDouble()) * Math.pow(ySupplier.getAsDouble(), 2),
                            deadbandSupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), deadbandSupplier.getAsDouble());

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX()
                                    * drive.getMaxLinearSpeedMetersPerSec()
                                    * percentSupplier.getAsDouble(),
                            linearVelocity.getY()
                                    * drive.getMaxLinearSpeedMetersPerSec()
                                    * percentSupplier.getAsDouble(),
                            omega * drive.getMaxAngularSpeedRadPerSec() * percentSupplier.getAsDouble());
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                },
                drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for
     * angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target,
     * or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier,
            DoubleSupplier deadbandSupplier) {

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ControlConstants.kPAngle,
                0.0,
                ControlConstants.kDAngle,
                new TrapezoidProfile.Constraints(
                        ControlConstants.angleMaxVelocity, ControlConstants.angleMaxAccel));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                            xSupplier.getAsDouble(),
                            ySupplier.getAsDouble(),
                            deadbandSupplier.getAsDouble());

                    // Calculate angular speed
                    double omega = angleController.calculate(
                            drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);
                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                },
                drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    public static Command driveForTime(Drive drive, double xSpeed, double ySpeed, double seconds) {
        return Commands.run(
                () -> {
                    drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, 0));
                },
                drive)
                .withTimeout(seconds)
                .andThen(
                        Commands.run(
                                () -> {
                                    drive.stopWithX();
                                }));
    }

    public static Command driveToRelativePosition(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier) {

        double xSetpoint = drive.getPose().getX() + xSupplier.getAsDouble();
        double ySetpoint = drive.getPose().getY() + ySupplier.getAsDouble();

        return Commands.run(
                () -> {
                    Pose2d pose = drive.getPose();

                    ChassisSpeeds speeds = new ChassisSpeeds(
                            xPID.calculate(pose.getX(), xSetpoint),
                            yPID.calculate(pose.getY(), ySetpoint),
                            headingPID.calculate(
                                    pose.getRotation().getRadians(), rotationSupplier.get().getRadians()));

                    drive.runVelocity(speeds);
                },
                drive)
                .until(() -> (xPID.atSetpoint() && yPID.atSetpoint()))
                .andThen(Commands.run(() -> drive.stopWithX()));
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        }),

                // Allow modules to orient
                Commands.run(
                        () -> {
                            drive.runCharacterization(0.0);
                        },
                        drive)
                        .withTimeout(Characterization.ffDelay),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                        () -> {
                            double voltage = timer.get() * Characterization.ffRampRate;
                            drive.runCharacterization(voltage);
                            velocitySamples.add(drive.getFFCharacterizationVelocity());
                            voltageSamples.add(voltage);
                        },
                        drive)

                        // When cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Drive FF Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(Characterization.wheelRadiusRampRate);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(
                                () -> {
                                    limiter.reset(0.0);
                                }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(Characterization.wheelRadiusMaxVelocity);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(
                                () -> {
                                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRotation();
                                    state.gyroDelta = 0.0;
                                }),

                        // Update gyro delta
                        Commands.run(
                                () -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(
                                        () -> {
                                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                            }
                                            double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS)
                                                    / wheelDelta;

                                            NumberFormat formatter = new DecimalFormat("#0.000");
                                            System.out.println(
                                                    "********** Wheel Radius Characterization Results **********");
                                            System.out.println(
                                                    "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                            System.out.println(
                                                    "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                            System.out.println(
                                                    "\tWheel Radius: "
                                                            + formatter.format(wheelRadius)
                                                            + " meters, "
                                                            + formatter.format(Units.metersToInches(wheelRadius))
                                                            + " inches");
                                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
