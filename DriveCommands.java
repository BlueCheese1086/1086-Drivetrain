package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants.Characterization;
import frc.robot.util.AdjustableValues;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

public class DriveCommands {
    private static final PIDController xController = new PIDController(
        AdjustableValues.getNumber("X_kP"),
        AdjustableValues.getNumber("X_kI"),
        AdjustableValues.getNumber("X_kD"));

    private static final PIDController yController = new PIDController(
        AdjustableValues.getNumber("Y_kP"),
        AdjustableValues.getNumber("Y_kI"),
        AdjustableValues.getNumber("Y_kD"));

    private static final ProfiledPIDController thetaController = new ProfiledPIDController(
        AdjustableValues.getNumber("Theta_kP"),
        AdjustableValues.getNumber("Theta_kI"),
        AdjustableValues.getNumber("Theta_kD"),
        new TrapezoidProfile.Constraints(
            DriveConstants.maxLinearVelocity.in(MetersPerSecond),
            DriveConstants.maxLinearAcceleration.in(MetersPerSecondPerSecond)));

    private static final HolonomicDriveController holonomicController = new HolonomicDriveController(xController, yController, thetaController);

    static {
        xController.setTolerance(0.03);
        yController.setTolerance(0.03);
        thetaController.setTolerance(0.03);
        holonomicController.setTolerance(new Pose2d(0.03, 0.03, new Rotation2d(0.03)));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static void updatePIDs() {
        if (AdjustableValues.hasChanged("X_kP")) xController.setP(AdjustableValues.getNumber("X_kP"));
        if (AdjustableValues.hasChanged("X_kI")) xController.setI(AdjustableValues.getNumber("X_kI"));
        if (AdjustableValues.hasChanged("X_kD")) xController.setD(AdjustableValues.getNumber("X_kD"));

        if (AdjustableValues.hasChanged("Y_kP")) yController.setP(AdjustableValues.getNumber("Y_kP"));
        if (AdjustableValues.hasChanged("Y_kI")) yController.setI(AdjustableValues.getNumber("Y_kI"));
        if (AdjustableValues.hasChanged("Y_kD")) yController.setD(AdjustableValues.getNumber("Y_kD"));

        if (AdjustableValues.hasChanged("Theta_kP")) thetaController.setP(AdjustableValues.getNumber("Theta_kP"));
        if (AdjustableValues.hasChanged("Theta_kI")) thetaController.setI(AdjustableValues.getNumber("Theta_kI"));
        if (AdjustableValues.hasChanged("Theta_kD")) thetaController.setD(AdjustableValues.getNumber("Theta_kD"));
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and
     * angular velocities).
     */
    public static Command joystickDrive(
            Drive drive,
            Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> omegaSupplier,
            Supplier<Double> percentSupplier) {
        return Commands.run(() -> {
                // Getting supplier values
                double x = xSupplier.get();
                double y = ySupplier.get();
                double omega = omegaSupplier.get();
                double percent = percentSupplier.get();

                // Applying deadbands
                x = MathUtil.applyDeadband(x, Constants.deadband);
                y = MathUtil.applyDeadband(y, Constants.deadband);
                omega = MathUtil.applyDeadband(omega, Constants.deadband);

                // Squaring inputs for more precise control
                x = Math.copySign(x * x, x);
                y = Math.copySign(y * y, y);
                omega = Math.copySign(omega * omega, omega);

                // Checking whether or not to flip the speeds
                boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

                // Converting to field relative speeds & send command
                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        x * DriveConstants.maxLinearVelocity.in(MetersPerSecond) * percent,
                        y * DriveConstants.maxLinearVelocity.in(MetersPerSecond) * percent,
                        omega * DriveConstants.maxAngularVelocity.in(RadiansPerSecond) * percent,
                        isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());

                // Driving the robot
                drive.runVelocity(speeds);
            },drive);
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
            Supplier<Double> xSupplier, Supplier<Double> ySupplier,
            Supplier<Rotation2d> rotationSupplier,
            Supplier<Double> deadbandSupplier,
            Supplier<Double> percentSupplier) {

        // Construct command
        return Commands.run(() -> {
                // Getting supplier values
                double x = xSupplier.get();
                double y = ySupplier.get();
                double deadband = deadbandSupplier.get();
                double percent = percentSupplier.get();

                // Applying deadbands
                x = MathUtil.applyDeadband(x, deadband);
                y = MathUtil.applyDeadband(y, deadband);

                // Squaring inputs for more precise control
                x = Math.copySign(x * x, x);
                y = Math.copySign(y * y, y);

                // Calculate angular speed
                double omega = thetaController.calculate(drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

                // Checking whether or not to flip the speeds
                boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

                // Convert to field relative speeds & send command
                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        x * DriveConstants.maxLinearVelocity.in(MetersPerSecond) * percent,
                        y * DriveConstants.maxLinearVelocity.in(MetersPerSecond) * percent,
                        omega * DriveConstants.maxAngularVelocity.in(RadiansPerSecond) * percent,
                        drive.getRotation().plus(new Rotation2d(isFlipped ? Math.PI : 0)));

                drive.runVelocity(speeds);
            },
            drive)

            // Reset PID controller when command starts
            .beforeStarting(() -> thetaController.reset(drive.getRotation().getRadians()));
    }

    public static Command driveForTime(Drive drive, double xSpeed, double ySpeed, double omega, double seconds) {
        return Commands.run(() -> drive.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, omega)), drive)
            .withTimeout(seconds)
            .andThen(() -> drive.stopWithX());
    }

    public static Command pidDriveToRelativePose(Drive drive, Translation2d translation) {
        Pose2d curPose = drive.getPose();

        xController.setSetpoint(curPose.getX() + translation.getX());
        yController.setSetpoint(curPose.getY() + translation.getY());
        thetaController.setGoal(new TrapezoidProfile.State(curPose.getRotation().plus(translation.getAngle()).getRadians(), 0));

        return Commands.run(() -> {
                Pose2d pose = drive.getPose();

                ChassisSpeeds speeds = new ChassisSpeeds(
                        xController.calculate(pose.getX()),
                        yController.calculate(pose.getY()),
                        thetaController.calculate(pose.getRotation().getRadians()));

                drive.runVelocity(speeds);
            },drive)
            .until(() -> (xController.atSetpoint() && yController.atSetpoint() && thetaController.atGoal()))
            .andThen(() -> drive.stopWithX());
    }

    public static Command pidDriveToPose(Drive drive, Pose2d goal) {
        xController.setSetpoint(goal.getX());
        yController.setSetpoint(goal.getY());
        thetaController.setGoal(new TrapezoidProfile.State(goal.getRotation().getRadians(), 0));

        return Commands.run(() -> {
                Pose2d pose = drive.getPose();

                ChassisSpeeds speeds = new ChassisSpeeds(
                        xController.calculate(pose.getX()),
                        yController.calculate(pose.getY()),
                        thetaController.calculate(pose.getRotation().getRadians()));

                drive.runVelocity(speeds);
            },drive)
            .until(() -> (xController.atSetpoint() && yController.atSetpoint() && thetaController.atGoal()))
            .andThen(() -> drive.stopWithX());
    }

    public static Command wpilibTrajToPose(Drive drive, Pose2d goal) {
        TrajectoryConfig config = new TrajectoryConfig(DriveConstants.maxLinearVelocity, DriveConstants.maxLinearAcceleration);
        config.setStartVelocity(Math.hypot(drive.getSpeed().vxMetersPerSecond, drive.getSpeed().vyMetersPerSecond));
        config.setEndVelocity(0.0);

        Trajectory traj = TrajectoryGenerator.generateTrajectory(drive.getPose(), List.of(), goal, config);
        Timer timer = new Timer();

        return Commands.run(() -> timer.start()).andThen(() -> {
            Trajectory.State state = traj.sample(timer.get());
            
            ChassisSpeeds speeds = holonomicController.calculate(drive.getPose(), state, state.poseMeters.getRotation());

            drive.runVelocity(speeds);
        })
        .until(() -> {
            Transform2d error = new Transform2d(goal, drive.getPose());

            return error.getX() < 0.3 && error.getY() < 0.3 && error.getRotation().getRadians() < 0.3;
        });
    }

    public static Command ppToPose(Drive drive, Pose2d goal, BooleanSupplier override) {
        return AutoBuilder.pathfindToPose(goal, DriveConstants.ppConstraints).until(override);
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
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(
                    () -> drive.runCharacterization(0.0), drive)
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
                    }, drive)

                    // When cancelled, calculate and print results
                    .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0;
                            double sumY = 0;
                            double sumXY = 0;
                            double sumX2 = 0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                            System.out.println("********** Drive FF Characterization Results **********");
                            System.out.printf("\tkS: #%.5f\n", kS);
                            System.out.printf("\tkV: #%.5f\n", kV);
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
                        Commands.runOnce(() -> limiter.reset(0.0)),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(Characterization.wheelRadiusMaxVelocity);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                }, drive)),

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
                                            double wheelRadius = (state.gyroDelta * DriveConstants.robotRadius.in(Meters)) / wheelDelta;

                                            System.out.printf("********** Wheel Radius Characterization Results **********\n");
                                            System.out.printf("\tWheel Delta: #%.3f radians\n", wheelDelta);
                                            System.out.printf("\tGyro Delta: #%.3f radians\n", state.gyroDelta);
                                            System.out.printf("\tWheel Radius: #%.3f meters, #%.3f inches", wheelRadius, Units.inchesToMeters(wheelRadius));
                                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}