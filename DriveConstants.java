package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.RobotMap;

public class DriveConstants {
    public static final Distance robotWidth = Inches.of(22.75);
    public static final Distance robotLength = Inches.of(22.75);

    public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(4.73);
    public static final LinearAcceleration maxLinearAcceleration = MetersPerSecondPerSecond.of(3);

    public static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(3 * Math.PI);
    public static final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(Math.PI);

    public static final Mass robotMass = Kilograms.of(50);
    public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(6.8);

    public static final Distance wheelRadius = Inches.of(1.931);

    public static final Current driveCurrentLimit = Amps.of(60);
    public static final Current steerCurrentLimit = Amps.of(30);

    public static final double kPDriveDefault = RobotBase.isReal() ? 1       : 0.3;
    public static final double kIDriveDefault = RobotBase.isReal() ? 0       : 0;
    public static final double kDDriveDefault = RobotBase.isReal() ? 0       : 0.01;
    public static final double kSDriveDefault = RobotBase.isReal() ? 0.06241 : 0;
    public static final double kVDriveDefault = RobotBase.isReal() ? 0.30278 : 0;

    public static final double kPSteerDefault = RobotBase.isReal() ? 1    : 5;
    public static final double kISteerDefault = RobotBase.isReal() ? 0    : 0;
    public static final double kDSteerDefault = RobotBase.isReal() ? 0.5  : 0;
    public static final double kSSteerDefault = RobotBase.isReal() ? 0.1  : 0;
    public static final double kVSteerDefault = RobotBase.isReal() ? 2.66 : 0;

    public static final double driveGearRatio = 5.14;
    public static final double steerGearRatio = 12.8;

    public static final double driveMOI = 0.025;
    public static final double steerMOI = 0.004;

    public static final Translation2d flModuleOffset = new Translation2d(robotWidth.div( 2), robotLength.div( 2));
    public static final Translation2d frModuleOffset = new Translation2d(robotWidth.div( 2), robotLength.div(-2));
    public static final Translation2d blModuleOffset = new Translation2d(robotWidth.div(-2), robotLength.div( 2));
    public static final Translation2d brModuleOffset = new Translation2d(robotWidth.div(-2), robotLength.div(-2));

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
        new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
        new SwerveModuleState(0, Rotation2d.fromDegrees( 135)),
        new SwerveModuleState(0, Rotation2d.fromDegrees( 135)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-135))
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

    // Kraken X44 DCMotor instance
    public static final DCMotor krakenX44 = new DCMotor(12, 4.05, 275, 1.4, 788.54, 1);
}
