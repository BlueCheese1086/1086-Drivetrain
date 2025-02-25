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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDValues;
import frc.robot.subsystems.gyro.Gyro;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
    private Gyro gyro;

    private ModuleIO[] modules;
    private SwerveModuleState[] states;
    private SwerveModulePosition[] positions;

    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

    private PIDController xController = new PIDController(10, 0, 0);
    private PIDController yController = new PIDController(10, 0, 0);
    private ProfiledPIDController headingController = new ProfiledPIDController(7.5, 0, 0, new TrapezoidProfile.Constraints(DriveConstants.maxAngularVelocity.in(RadiansPerSecond), DriveConstants.maxAngularAcceleration.in(RadiansPerSecondPerSecond)));
    private HolonomicDriveController controller = new HolonomicDriveController(xController, yController, headingController);

    private TrajectoryConfig trajectoryConfig;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) new Drivetrain(new ModuleIOSim(0), new ModuleIOSim(1), new ModuleIOSim(2), new ModuleIOSim(3));

        return instance;
    }

    public Drivetrain(ModuleIO... modules) {
        if (instance != null) return;

        this.gyro = Gyro.getInstance();
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

        // Initializing the trajectory config
        trajectoryConfig = new TrajectoryConfig(DriveConstants.maxLinearVelocity, DriveConstants.maxLinearAcceleration);
        trajectoryConfig.setEndVelocity(MetersPerSecond.zero());
        trajectoryConfig.setKinematics(kinematics);

        // Configuring Pathplanner
        AutoBuilder.configure(this::getPose, this::resetPose, this::getSpeeds, this::drive,
            new PPHolonomicDriveController(
                new PIDConstants(PIDValues.kPDrive, PIDValues.kIDrive, PIDValues.kDDrive),
                new PIDConstants(PIDValues.kPSteer, PIDValues.kISteer, PIDValues.kDSteer)
            ),
            new RobotConfig(
                DriveConstants.robotMass, DriveConstants.robotMOI,
                new ModuleConfig(DriveConstants.wheelRadius, DriveConstants.maxLinearVelocity, 1, DCMotor.getKrakenX60(1), DriveConstants.driveCurrentLimit, 2), 
                DriveConstants.translations),
            () -> (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)),
            this);

        // Configuring Choreo
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        instance = this;
    }

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
     * This function 
     * @param log
     */
    public void sysIdLog(SysIdRoutineLog log) {
        log.motor("FLDrive")
            .linearVelocity(MetersPerSecond.of(modules[0].getState().speedMetersPerSecond))
            .linearPosition(Meters.of(modules[0].getPosition().distanceMeters))
            .voltage(modules[0].getDriveVoltage());

        log.motor("FrDrive")
            .linearVelocity(MetersPerSecond.of(modules[1].getState().speedMetersPerSecond))
            .linearPosition(Meters.of(modules[1].getPosition().distanceMeters));

        log.motor("BLDrive")
            .linearVelocity(MetersPerSecond.of(modules[2].getState().speedMetersPerSecond))
            .linearPosition(Meters.of(modules[2].getPosition().distanceMeters));

        log.motor("BRDrive")
            .linearVelocity(MetersPerSecond.of(modules[3].getState().speedMetersPerSecond))
            .linearPosition(Meters.of(modules[3].getPosition().distanceMeters));
    }

    @Override
    public void periodic() {
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
            positions[i] = modules[i].getPosition();
            modules[i].updateInputs();
        }

        poseEstimator.update(getHeading(), positions);

        Logger.recordOutput("/Drivetrain/States/Actual", states);
        Logger.recordOutput("/Drivetrain/Positions/Actual", positions);

        Logger.recordOutput("/Drivetrain/EstimatedPose", poseEstimator.getEstimatedPosition());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPosition(getHeading(), positions, newPose);
    }

    public Rotation2d getHeading() {
        if (gyro == null) return new Rotation2d();

        return gyro.getHeading();
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(states);
    }

    public void drive(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxLinearVelocity);

        for (int i = 0; i < modules.length; i++) {
            desiredStates[i].optimize(modules[i].getAngle());
            desiredStates[i].cosineScale(modules[i].getAngle());

            modules[i].setState(desiredStates[i]);
        }

        Logger.recordOutput("/Drivetrain/States/Setpoint", desiredStates);
        Logger.recordOutput("/Drivetrain/Speeds/Setpoint", speeds);
    }

    public void pathfindToPose(Pose2d goalPose, Translation2d... translations) {
        List<Translation2d> translationList = Arrays.asList(translations);

        Trajectory traj = TrajectoryGenerator.generateTrajectory(getPose(), translationList, goalPose, trajectoryConfig);

        traj.getTotalTimeSeconds();
        Trajectory.State goalState = traj.sample(0.02);

        drive(controller.calculate(getPose(), goalState, goalPose.getRotation()));
    }

    public void followTrajectory(SwerveSample sample) {
        pathfindToPose(sample.getPose());
    }

    public void xStates() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(DriveConstants.xStates[i]);
        }
    }
}