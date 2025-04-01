// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.util.VisionResult;
import frc.robot.util.AdjustableValues;
import frc.robot.util.PoseAllignment;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    // DriveConstants doesn't include these constants, so they are declared locally
    private final PoseAllignment poseAllignment = new PoseAllignment();
    static final double ODOMETRY_FREQUENCY = new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD()
            ? 250.0
            : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    static final Lock odometryLock = new ReentrantLock();
    private List<Pose2d> poseHistory = new ArrayList<>();
    private final GyroIO gyroIO;
    private final Vision vision;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);
    private RobotConfig config;
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, new Pose2d(0, 0, new Rotation2d()));

    private PIDController autoXPID = new PIDController(AdjustableValues.getNumber("xkP"), 0.0, 0.0);
    private PIDController autoYPID = new PIDController(AdjustableValues.getNumber("yKP"), 0.0, 0.0);
    private PIDController autoHeadingPID = new PIDController(AdjustableValues.getNumber("rotKP"), 0.0, 0.0);
    private ProfiledPIDController trajHeading = new ProfiledPIDController(AdjustableValues.getNumber("rotkP"), 0.0, 0.0,
            new Constraints(3.5, 3.5));

    public PathConstraints constraints = new PathConstraints(5.25, 4.75, Units.degreesToRadians(640),
            Units.degreesToRadians(820));

    public TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            LinearVelocity.ofBaseUnits(4.30, MetersPerSecond),
            LinearAcceleration.ofBaseUnits(4.99, MetersPerSecondPerSecond))
            .setKinematics(kinematics)
            .setReversed(false)
            .setStartVelocity(0.3)
            .setEndVelocity(3.0);

    public HolonomicDriveController trajDriveController = new HolonomicDriveController(autoXPID, autoYPID, trajHeading);

    public Drive(
            GyroIO gyroIO,
            Vision vision,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {

        this.gyroIO = gyroIO;
        this.vision = vision;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                (speeds) -> runVelocity(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(AdjustableValues.getNumber("xkP"), 0.0, 0.0),
                        new PIDConstants(AdjustableValues.getNumber("rotkP"), 0.0, 0.0)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        for (VisionResult result : vision.getUnreadResults()) {
            poseEstimator.addVisionMeasurement(result.getPose2d(), result.getTimestamp());
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
                checkTip();
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

            if (AdjustableValues.hasChanged("xkP"))
                autoXPID.setP(AdjustableValues.getNumber("xkP"));
            if (AdjustableValues.hasChanged("ykP"))
                autoXPID.setP(AdjustableValues.getNumber("ykP"));
            if (AdjustableValues.hasChanged("rotkP"))
                autoXPID.setP(AdjustableValues.getNumber("rotkP"));
        }

        // Update gyro alert
        // gyroDisconnectedAlert.set(!gyroInputs.connected && Robot.isReal() !=
        // Robot.isSimulation());
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void checkTip() {
        if (gyroInputs.tilt >= 0.0) {
            Logger.recordOutput("Drive/Debug/Gyro/Tilt", gyroInputs.tilt);
            stop();
        }
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public void logPose() {
        Pose2d newPose = poseEstimator.getEstimatedPosition();
        poseHistory.add(newPose);
        logHistory();
    }

    public void logHistory() {
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            Logger.recordOutput("Drive/Pose " + i, pose);
        }
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native
     * units).
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    public ChassisSpeeds getSpeed() {
        ChassisSpeeds currentSpeed = kinematics.toChassisSpeeds(getModuleStates());
        return currentSpeed;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
                new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
                new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    public Rotation2d getReefAngle() {
        return new Rotation2d();
    }

    public ChassisSpeeds robotRelativeSpeed() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
    }

    @AutoLogOutput(key = "Drive/Debug/PathPlanner/LeftAlign")
    public Pose2d autoLeftPose(boolean alliance) {
        Pose2d targetPose;
        if (alliance == true) {
            targetPose = poseEstimator.getEstimatedPosition().nearest(poseAllignment.blueLeft);
        } else {
            targetPose = poseEstimator.getEstimatedPosition().nearest(poseAllignment.redLeft);
        }
        Logger.recordOutput("Drive/Deubg/PathPlanner/Pose/LeftTargetPose", targetPose);
        return targetPose;
    }

    @AutoLogOutput(key = "Drive/Debug/PathPlanner/RightAlign")
    public Pose2d autoRightPose(boolean alliance) {
        Pose2d targetPose;
        if (alliance == true) {
            targetPose = poseEstimator.getEstimatedPosition().nearest(poseAllignment.blueRight);
        } else {
            targetPose = poseEstimator.getEstimatedPosition().nearest(poseAllignment.redRight);
        }
        Logger.recordOutput("Drive/Deubg/PathPlanner/Pose/RightTargetPose", targetPose);
        return targetPose;
    }

    public Pose2d autoHP(boolean alliance) {
        Pose2d targetPose;
        if (alliance == true) {
            targetPose = poseEstimator.getEstimatedPosition().nearest(poseAllignment.HPBlue);
        } else {
            targetPose = poseEstimator.getEstimatedPosition().nearest(poseAllignment.HPRed);
        }

        Logger.recordOutput("Drive/Debug/PathPlanner/Pose/HPPose", targetPose);
        return targetPose;
    }

    public Command followTraj() {
        return new RunCommand(
                () -> {
                    Pose2d robotPose = poseEstimator.getEstimatedPosition();
                    Pose2d target = robotPose.nearest(poseAllignment.redLeft);

                    Trajectory traj = TrajectoryGenerator.generateTrajectory(
                            robotPose, List.of(), target, trajectoryConfig);

                    Trajectory.State desiredState = traj.sample(traj.getTotalTimeSeconds());
                    Logger.recordOutput("Drive/PID/Align", target);
                    ChassisSpeeds zoom = trajDriveController.calculate(robotPose, desiredState, target.getRotation());
                    this.runVelocity(zoom);
                    if (trajDriveController.atReference()) {
                        this.runVelocity(new ChassisSpeeds());
                    }
                },
                this);
    }
}
