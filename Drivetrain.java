package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.AdjustableValues;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.util.VisionResult;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
    private ModuleIO[] modules;
    private SwerveModuleState[] states;
    private SwerveModulePosition[] positions;

    private PIDController xController = new PIDController(AdjustableValues.getNumber("X_kP"),
                                                          AdjustableValues.getNumber("X_kI"),
                                                          AdjustableValues.getNumber("X_kD"));

    private PIDController yController = new PIDController(AdjustableValues.getNumber("Y_kP"),
                                                          AdjustableValues.getNumber("Y_kI"),
                                                          AdjustableValues.getNumber("Y_kD"));

    private ProfiledPIDController thetaController = new ProfiledPIDController(AdjustableValues.getNumber("Theta_kP"),
                                                                              AdjustableValues.getNumber("Theta_kI"),
                                                                              AdjustableValues.getNumber("Theta_kD"),
                                                                              new TrapezoidProfile.Constraints(
                                                                                  DriveConstants.maxAngularVelocity.in(RadiansPerSecond),
                                                                                  DriveConstants.maxAngularAcceleration.in(RadiansPerSecondPerSecond)));

    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;
    private HolonomicDriveController holonomicController = new HolonomicDriveController(xController, yController, thetaController);

    // Subsystem depencies
    private Gyro gyro;
    private Vision vision;

    // Configs for closed loop control
    private boolean headingLocked;
    private Rotation2d lockedAngle;

    private Rotation2d estimatedHeading = new Rotation2d();

    /**
     * Creates a new Drivetrain subsystem.
     * 
     * @param gyro The gyro instance to get heading from.
     * @param vision The vision instance to get pose estimates from.
     * @param modules The module IOs to drive on.
    */
    public Drivetrain(Gyro gyro, Vision vision, ModuleIO... modules) {
        System.out.println("Drivetrain initialized");
        // Saving subsystems
        this.gyro = gyro;
        this.vision = vision;

        this.modules = modules;
        this.states = new SwerveModuleState[modules.length];
        this.positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
            positions[i] = modules[i].getPosition();
        }

        /*
         *  BL | FL 
         *     |    
         * ---------
         *     |    
         *  BR | FR 
         */

        kinematics = new SwerveDriveKinematics(DriveConstants.translations);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getHeading(), positions, new Pose2d());

        // Configuring SysID
        new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::driveVolts, this::sysIdLog, this, "SwerveDrive"));

        // Configuring Pathplanner
        AutoBuilder.configure(this::getPose, this::resetPose, this::getSpeeds, this::drive,
            new PPHolonomicDriveController(
                new PIDConstants(DriveConstants.kPDriveDefault, DriveConstants.kIDriveDefault, DriveConstants.kDDriveDefault),
                new PIDConstants(DriveConstants.kPSteerDefault, DriveConstants.kISteerDefault, DriveConstants.kDSteerDefault)
            ),
            new RobotConfig(
                DriveConstants.robotMass, DriveConstants.robotMOI,
                new ModuleConfig(DriveConstants.wheelRadius, DriveConstants.maxLinearVelocity, 1, DCMotor.getKrakenX60(1).withReduction(DriveConstants.driveGearRatio), DriveConstants.driveCurrentLimit, 2), 
                DriveConstants.translations),
            () -> (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)),
            this);

        // Configuring Choreo
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /** Gets the point on the reef that is closest to the robot's current pose. */
    public Pose2d getClosestReefPoint() {
        Pose2d curPose = getPose();

        double[] distances = new double[12];

        distances[0]  = Constants.Poses.REEF_Side1Left.getTranslation().getDistance(curPose.getTranslation());
        distances[1]  = Constants.Poses.REEF_Side1Right.getTranslation().getDistance(curPose.getTranslation());
        distances[2]  = Constants.Poses.REEF_Side2Left.getTranslation().getDistance(curPose.getTranslation());
        distances[3]  = Constants.Poses.REEF_Side2Right.getTranslation().getDistance(curPose.getTranslation());
        distances[4]  = Constants.Poses.REEF_Side3Left.getTranslation().getDistance(curPose.getTranslation());
        distances[5]  = Constants.Poses.REEF_Side3Right.getTranslation().getDistance(curPose.getTranslation());
        distances[6]  = Constants.Poses.REEF_Side4Left.getTranslation().getDistance(curPose.getTranslation());
        distances[7]  = Constants.Poses.REEF_Side4Right.getTranslation().getDistance(curPose.getTranslation());
        distances[8]  = Constants.Poses.REEF_Side5Left.getTranslation().getDistance(curPose.getTranslation());
        distances[9]  = Constants.Poses.REEF_Side5Right.getTranslation().getDistance(curPose.getTranslation());
        distances[10] = Constants.Poses.REEF_Side6Left.getTranslation().getDistance(curPose.getTranslation());
        distances[11] = Constants.Poses.REEF_Side6Right.getTranslation().getDistance(curPose.getTranslation());

        int minDistIndex = 0;
        for (int i = 0; i < 12; i++) {
            if (distances[i] < distances[minDistIndex]) minDistIndex = i;
        }

        switch (minDistIndex) {
            case 0:  return Constants.Poses.REEF_Side1Left;
            case 1:  return Constants.Poses.REEF_Side1Right;
            case 2:  return Constants.Poses.REEF_Side2Left;
            case 3:  return Constants.Poses.REEF_Side2Right;
            case 4:  return Constants.Poses.REEF_Side3Left;
            case 5:  return Constants.Poses.REEF_Side3Right;
            case 6:  return Constants.Poses.REEF_Side4Left;
            case 7:  return Constants.Poses.REEF_Side4Right;
            case 8:  return Constants.Poses.REEF_Side5Left;
            case 9:  return Constants.Poses.REEF_Side5Right;
            case 10: return Constants.Poses.REEF_Side6Left;
            default: return Constants.Poses.REEF_Side6Right;
        }
    }

    /**
     * DO NOT USE FOR ANYTHING OTHER THAN SYSID!!!
     * THIS FUNCTION DOES NOT CONTROL THE TURN MOTOR
     * 
     * @param volts
     */
    public void driveVolts(Voltage volts) {

    }

    /**
     * This function logs values from sysId.
     * 
     * @param log The log structure to apply changes to.
     */
    public void sysIdLog(SysIdRoutineLog log) {
        log.motor("FLDrive")
            .linearVelocity(MetersPerSecond.of(modules[0].getState().speedMetersPerSecond))
            .linearPosition(Meters.of(modules[0].getPosition().distanceMeters))
            .voltage(modules[0].getDriveVoltage());

        log.motor("FRDrive")
            .linearVelocity(MetersPerSecond.of(modules[1].getState().speedMetersPerSecond))
            .linearPosition(Meters.of(modules[1].getPosition().distanceMeters));

        log.motor("BLDrive")
            .linearVelocity(MetersPerSecond.of(modules[2].getState().speedMetersPerSecond))
            .linearPosition(Meters.of(modules[2].getPosition().distanceMeters));

        log.motor("BRDrive")
            .linearVelocity(MetersPerSecond.of(modules[3].getState().speedMetersPerSecond))
            .linearPosition(Meters.of(modules[3].getPosition().distanceMeters));
    }

    /**
     * Runs once every tick the subsystem is active.
     * 
     * It updates the module IO inputs, the estimated pose, and the module states/positions.
     * It also logs multiple values specific to the drivetrain subsystem.
     */
    @Override
    public void periodic() {
        if (AdjustableValues.hasChanged("X_kP")) xController.setP(AdjustableValues.getNumber("X_kP"));
        if (AdjustableValues.hasChanged("X_kI")) xController.setI(AdjustableValues.getNumber("X_kI"));
        if (AdjustableValues.hasChanged("X_kD")) xController.setD(AdjustableValues.getNumber("X_kD"));

        if (AdjustableValues.hasChanged("Y_kP")) yController.setP(AdjustableValues.getNumber("Y_kP"));
        if (AdjustableValues.hasChanged("Y_kI")) yController.setI(AdjustableValues.getNumber("Y_kI"));
        if (AdjustableValues.hasChanged("Y_kD")) yController.setD(AdjustableValues.getNumber("Y_kD"));

        if (AdjustableValues.hasChanged("Theta_kP")) thetaController.setP(AdjustableValues.getNumber("Theta_kP"));
        if (AdjustableValues.hasChanged("Theta_kI")) thetaController.setI(AdjustableValues.getNumber("Theta_kI"));
        if (AdjustableValues.hasChanged("Theta_kD")) thetaController.setD(AdjustableValues.getNumber("Theta_kD"));

        SwerveModulePosition[] oldPositions = positions.clone();

        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs();
            states[i] = modules[i].getState();
            positions[i] = modules[i].getPosition();
        }

        if (gyro == null) {
            SwerveModulePosition[] deltas = new SwerveModulePosition[4];

            for (int i = 0; i < 4; i++) {
                deltas[i] = new SwerveModulePosition(positions[i].distanceMeters - oldPositions[i].distanceMeters, positions[i].angle);
            }

            Twist2d twist = kinematics.toTwist2d(deltas);

            estimatedHeading = estimatedHeading.plus(new Rotation2d(twist.dtheta));
        }

        poseEstimator.update(getHeading(), positions);

        for (VisionResult result : vision.getUnreadResults()) {
            poseEstimator.addVisionMeasurement(result.getPose2d(), result.getTimestamp());
        }

        Logger.recordOutput("/Subsystems/Drivetrain/HeadingLocked", headingLocked);
        Logger.recordOutput("/Subsystems/Drivetrain/HeadingSetpoint", lockedAngle);

        Logger.recordOutput("/Subsystems/Drivetrain/States/Actual", states);
        Logger.recordOutput("/Subsystems/Drivetrain/Positions/Actual", positions);

        Logger.recordOutput("/Subsystems/Drivetrain/RobotPose", poseEstimator.getEstimatedPosition());
    }

    /** Gets the current pose. */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the pose
     * 
     * @param newPose The new pose to go to.
     */
    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPosition(getHeading(), positions, newPose);
    }

    /** Gets the current heading. */
    public Rotation2d getHeading() {
        if (gyro == null) return estimatedHeading;

        return new Rotation2d(gyro.getYaw());
    }

    /** Gets the current wheel speeds. */
    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(states);
    }

    /**
     * Drives the robot according to some ChassisSpeeds.
     * 
     * If the heading is locked, omega is ignored.
     */
    public void drive(ChassisSpeeds speeds) {
        if (headingLocked) {
            Rotation2d angle = (lockedAngle == null) ? getHeading() : lockedAngle;
            speeds.omegaRadiansPerSecond = getSpeeds().omegaRadiansPerSecond + thetaController.calculate(getHeading().getRadians(), angle.getRadians());
        }

        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxLinearVelocity);

        for (int i = 0; i < modules.length; i++) {
            desiredStates[i].optimize(modules[i].getAngle());
            desiredStates[i].cosineScale(modules[i].getAngle());

            modules[i].setState(desiredStates[i]);
        }

        Logger.recordOutput("/Subsystems/Drivetrain/States/Setpoint", desiredStates);
        Logger.recordOutput("/Subsystems/Drivetrain/Speeds/Setpoint", speeds);
    }

    /** Locks the heading */
    public void lockHeading() {
        headingLocked = true;
    }

    /**
     * Locks the heading.
     * 
     * @param angle The angle to lock the heading to.
     */
    public void lockHeading(Rotation2d angle) {
        setLockedAngle(angle);
    }

    /** Unlocks the heading. */
    public void unlockHeading() {
        headingLocked = false;
    }

    /**
     * Gets the angle that the robot is locked onto.
     * 
     * Returns null if the heading is not locked.
     */
    public Rotation2d getLockedAngle() {
        if (headingLocked) return lockedAngle;

        return null;
    }

    /** Sets the angle that the robot will lock to. */
    public void setLockedAngle(Rotation2d angle) {
        lockedAngle = angle;
    }

    /** Gets the kinematics of the robot. */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
    
    /** Gets the holonomic controller for the robot. */
    public HolonomicDriveController getController() {
        return holonomicController;
    }

    /** Follows a choreo trajectory. */
    public void followTrajectory(SwerveSample sample) {
        double velocity = Math.hypot(sample.vx, sample.vy);
        double acceleration = Math.hypot(sample.ax, sample.ay);

        Trajectory.State state = new Trajectory.State(sample.getTimestamp(), velocity, acceleration, sample.getPose(), acceleration / Math.pow(velocity, 2));
        
        drive(holonomicController.calculate(getPose(), state, Rotation2d.fromRadians(sample.heading)));
    }

    /** Sets the states of each module to an "X" pattern. */
    public void xStates() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(DriveConstants.xStates[i]);
        }
    }
}