package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.RobotMap;

public class DriveConstants {
    public static final double robotWidth = Units.inchesToMeters(22.75); // Meters
    public static final double robotLength = Units.inchesToMeters(22.75); // Meters

    public static final double driveXPercent = 1;
    public static final double driveYPercent = 1;
    public static final double steerPercent = 1;
    public static final double precisionPercent = 0.2;

    public static final double maxLinearVelocity = 4.73; // Meters / Second
    public static final double maxLinearAcceleration = 3; // Meters / Second^2

    public static final double maxAngularVelocity = 3 * Math.PI; // Radians / Second
    public static final double maxAngularAcceleration = Math.PI; // Radians / Second^2

    public static final double robotMass = 50; // Kilograms
    public static final double robotMOI = 6.8; // Kilograms^2 * Meters

    public static final double wheelRadius = Units.inchesToMeters(1.931); // Meters

    public static final double driveCurrentLimit = 60; // Amps
    public static final double steerCurrentLimit = 30; // Amps

    public static final double kPDriveDefault = RobotBase.isReal() ? 1       : 0.5;
    public static final double kIDriveDefault = RobotBase.isReal() ? 0       : 0.0;
    public static final double kDDriveDefault = RobotBase.isReal() ? 0       : 0.0;
    public static final double kSDriveDefault = RobotBase.isReal() ? 0.06241 : 1.5;
    public static final double kVDriveDefault = RobotBase.isReal() ? 0.30278 : 0.08;

    public static final double kPSteerDefault = RobotBase.isReal() ? 1    : 5;
    public static final double kISteerDefault = RobotBase.isReal() ? 0    : 0;
    public static final double kDSteerDefault = RobotBase.isReal() ? 0.5  : 0;
    public static final double kSSteerDefault = RobotBase.isReal() ? 0.1  : 0;
    public static final double kVSteerDefault = RobotBase.isReal() ? 2.66 : 0;

    public static final double driveGearRatio = 5.14;
    public static final double steerGearRatio = 12.8;

    public static final double driveMOI = 0.025; // Kilograms^2 * Meters
    public static final double steerMOI = 0.004; // Kilograms^2 * Meters

    public static final Translation2d flModuleOffset = new Translation2d( robotWidth / 2,  robotLength / 2);
    public static final Translation2d frModuleOffset = new Translation2d( robotWidth / 2, -robotLength / 2);
    public static final Translation2d blModuleOffset = new Translation2d(-robotWidth / 2,  robotLength / 2);
    public static final Translation2d brModuleOffset = new Translation2d(-robotWidth / 2, -robotLength / 2);

    public static final double flEncoderOffset = 0.566894531; // Rotations
    public static final double frEncoderOffset = 0.388427734; // Rotations
    public static final double blEncoderOffset = 0.114257813; // Rotations
    public static final double brEncoderOffset = 0.085205078; // Rotations

    // Arrays for easy configuration access
    public static final Translation2d[] translations = { flModuleOffset, frModuleOffset, blModuleOffset, brModuleOffset };

    public static final double[][] moduleConfigs = {
        {RobotMap.DT_FLDrive, RobotMap.DT_FLSteer, RobotMap.DT_FLEncoder, DriveConstants.flEncoderOffset}, // FL: drive id, steer id, encoder id, encoder offset
        {RobotMap.DT_FRDrive, RobotMap.DT_FRSteer, RobotMap.DT_FREncoder, DriveConstants.frEncoderOffset}, // FR: drive id, steer id, encoder id, encoder offset
        {RobotMap.DT_BLDrive, RobotMap.DT_BLSteer, RobotMap.DT_BLEncoder, DriveConstants.blEncoderOffset}, // BL: drive id, steer id, encoder id, encoder offset
        {RobotMap.DT_BRDrive, RobotMap.DT_BRSteer, RobotMap.DT_BREncoder, DriveConstants.brEncoderOffset}  // BR: drive id, steer id, encoder id, encoder offset
    };

    public static final SwerveModuleState[] xStates = {
        new SwerveModuleState(0, Rotation2d.fromDegrees(-135)), // FL
        new SwerveModuleState(0, Rotation2d.fromDegrees( 135)), // FR
        new SwerveModuleState(0, Rotation2d.fromDegrees( 135)), // BL
        new SwerveModuleState(0, Rotation2d.fromDegrees(-135))  // BR
    };

    public static final double kPX = 7;
    public static final double kIX = 0;
    public static final double kDX = 0;

    public static final double kPY = 7;
    public static final double kIY = 0;
    public static final double kDY = 0;
    
    public static final double kPTheta = 15;
    public static final double kITheta = 0;
    public static final double kDTheta = 0;

    // Kraken X44 DCMotor instance.  Values are from their documentation.
    public static final DCMotor krakenX44 = new DCMotor(12, 4.05, 275, 1.4, 788.54, 1);
}
