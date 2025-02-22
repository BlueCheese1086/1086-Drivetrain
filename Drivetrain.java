package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.gyro.Gyro;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
    private Gyro gyro;

    private ModuleIO[] modules;
    private SwerveModuleState[] states;
    private SwerveModulePosition[] positions;

    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

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

        Translation2d[] translations = {
            DriveConstants.flModuleOffset,
            DriveConstants.frModuleOffset,
            DriveConstants.blModuleOffset,
            DriveConstants.brModuleOffset
        };

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

        kinematics = new SwerveDriveKinematics(translations);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getHeading(), positions, new Pose2d());

        // Configuring SysID
        new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::driveVolts, this::sysIdLog, this, "SwerveDrive"));

        // Configuring Pathplanner
        AutoBuilder.configure(this::getPose, this::resetPose, this::getSpeeds, this::drive,
            new PPHolonomicDriveController(
                new PIDConstants(
                    RobotBase.isReal() ? DriveConstants.kPDriveReal : DriveConstants.kPDriveSim,
                    RobotBase.isReal() ? DriveConstants.kIDriveReal : DriveConstants.kIDriveSim,
                    RobotBase.isReal() ? DriveConstants.kDDriveReal : DriveConstants.kDDriveSim),
                new PIDConstants(
                    RobotBase.isReal() ? DriveConstants.kPSteerReal : DriveConstants.kPSteerSim,
                    RobotBase.isReal() ? DriveConstants.kISteerReal : DriveConstants.kISteerSim,
                    RobotBase.isReal() ? DriveConstants.kDSteerReal : DriveConstants.kDSteerSim)
            ),
            new RobotConfig(
                DriveConstants.robotMass, DriveConstants.robotMOI,
                new ModuleConfig(DriveConstants.wheelRadius, DriveConstants.maxLinearVelocity, 1, DCMotor.getKrakenX60(1), DriveConstants.driveCurrentLimit, 2), 
                translations),
            () -> (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)),
            this);

        instance = this;
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

        Logger.recordOutput("Drivetrain/ActualStates", states);
        Logger.recordOutput("Drivetrain/ActualPositions", positions);

        Logger.recordOutput("Drivetrain/EstimatedPose", poseEstimator.getEstimatedPosition());
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

        Logger.recordOutput("Drivetrain/DesiredStates", desiredStates);
        Logger.recordOutput("Drivetrain/DesiredSpeeds", speeds);
    }
}