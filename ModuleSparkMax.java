
package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.TurboLogger;

public class ModuleSparkMax extends Module {
    private int moduleId;

    private SparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkClosedLoopController driveController;

    private SparkMax steerMotor;
    private RelativeEncoder steerEncoder;
    private SparkClosedLoopController steerController;

    private CANcoder absEncoder;
    private double encoderOffset;

    private SimpleMotorFeedforward driveFFController;
    private SimpleMotorFeedforward steerFFController;

    /**
     * Creates a new ModuleIO with SparkMAX motors.
     *
     * @param moduleId The module id used for logging and getting configs.
     */
    public ModuleSparkMax(int moduleId) {
        this.moduleId = moduleId;

        absEncoder = new CANcoder((int) DriveConstants.moduleConfigs[moduleId][2]);
        encoderOffset = DriveConstants.moduleConfigs[moduleId][3];

        driveMotor = new SparkMax((int) DriveConstants.moduleConfigs[moduleId][0], MotorType.kBrushless);
        steerMotor = new SparkMax((int) DriveConstants.moduleConfigs[moduleId][1], MotorType.kBrushless);

        driveFFController = new SimpleMotorFeedforward(TurboLogger.get("Drive_kS_" + moduleId, DriveConstants.kSDriveDefault), TurboLogger.get("Drive_kV_" + moduleId, DriveConstants.kVDriveDefault), 0, 0.02);
        steerFFController = new SimpleMotorFeedforward(TurboLogger.get("Steer_kS_" + moduleId, DriveConstants.kSSteerDefault), TurboLogger.get("Steer_kV_" + moduleId, DriveConstants.kVSteerDefault), 0, 0.02);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.closedLoop.p(TurboLogger.get("Drive_kP_" + moduleId, DriveConstants.kPDriveDefault), ClosedLoopSlot.kSlot0);
        driveConfig.closedLoop.i(TurboLogger.get("Drive_kI_" + moduleId, DriveConstants.kIDriveDefault), ClosedLoopSlot.kSlot0);
        driveConfig.closedLoop.d(TurboLogger.get("Drive_kD_" + moduleId, DriveConstants.kDDriveDefault), ClosedLoopSlot.kSlot0);
        driveConfig.encoder.positionConversionFactor(DriveConstants.wheelRadius.in(Meters) / DriveConstants.driveGearRatio);
        driveConfig.encoder.velocityConversionFactor(DriveConstants.wheelRadius.in(Meters) / DriveConstants.driveGearRatio / 60);
        driveConfig.inverted(false);
        driveConfig.idleMode(IdleMode.kCoast);
        driveConfig.smartCurrentLimit((int) DriveConstants.driveCurrentLimit.in(Amps));

        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.closedLoop.p(TurboLogger.get("Steer_kP_" + moduleId, DriveConstants.kPSteerDefault), ClosedLoopSlot.kSlot0);
        steerConfig.closedLoop.i(TurboLogger.get("Steer_kI_" + moduleId, DriveConstants.kISteerDefault), ClosedLoopSlot.kSlot0);
        steerConfig.closedLoop.d(TurboLogger.get("Steer_kD_" + moduleId, DriveConstants.kDSteerDefault), ClosedLoopSlot.kSlot0);
        steerConfig.closedLoop.positionWrappingEnabled(true);
        steerConfig.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);
        steerConfig.encoder.positionConversionFactor(2 * Math.PI / DriveConstants.steerGearRatio);
        steerConfig.encoder.velocityConversionFactor(2 * Math.PI / DriveConstants.steerGearRatio / 60);
        steerConfig.inverted(false);
        steerConfig.idleMode(IdleMode.kCoast);
        steerConfig.smartCurrentLimit((int) DriveConstants.steerCurrentLimit.in(Amps));

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        steerEncoder.setPosition(absEncoder.getAbsolutePosition().getValue().in(Rotations) - encoderOffset);

        driveController = driveMotor.getClosedLoopController();
        steerController = steerMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        SparkMaxConfig drivePIDConfig = new SparkMaxConfig();
        if (TurboLogger.hasChanged("Drive_kP_" + moduleId)) drivePIDConfig.closedLoop.p(TurboLogger.get("Drive_kP_" + moduleId, DriveConstants.kPDriveDefault));
        if (TurboLogger.hasChanged("Drive_kI_" + moduleId)) drivePIDConfig.closedLoop.i(TurboLogger.get("Drive_kI_" + moduleId, DriveConstants.kIDriveDefault));
        if (TurboLogger.hasChanged("Drive_kD_" + moduleId)) drivePIDConfig.closedLoop.d(TurboLogger.get("Drive_kD_" + moduleId, DriveConstants.kDDriveDefault));
        if (TurboLogger.hasChanged("Drive_kS_" + moduleId)) driveFFController.setKs(TurboLogger.get("Drive_kS_" + moduleId, DriveConstants.kSDriveDefault));
        if (TurboLogger.hasChanged("Drive_kV_" + moduleId)) driveFFController.setKv(TurboLogger.get("Drive_kV_" + moduleId, DriveConstants.kVDriveDefault));
        if (!drivePIDConfig.closedLoop.flatten().equals(new ClosedLoopConfig().flatten())) driveMotor.configure(drivePIDConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig steerPIDConfig = new SparkMaxConfig();
        if (TurboLogger.hasChanged("Steer_kP_" + moduleId)) steerPIDConfig.closedLoop.p(TurboLogger.get("Steer_kP_" + moduleId, DriveConstants.kPSteerDefault));
        if (TurboLogger.hasChanged("Steer_kI_" + moduleId)) steerPIDConfig.closedLoop.i(TurboLogger.get("Steer_kI_" + moduleId, DriveConstants.kISteerDefault));
        if (TurboLogger.hasChanged("Steer_kD_" + moduleId)) steerPIDConfig.closedLoop.d(TurboLogger.get("Steer_kD_" + moduleId, DriveConstants.kDSteerDefault));
        if (TurboLogger.hasChanged("Steer_kS_" + moduleId)) steerFFController.setKs(TurboLogger.get("Steer_kS_" + moduleId, DriveConstants.kSSteerDefault));
        if (TurboLogger.hasChanged("Steer_kV_" + moduleId)) steerFFController.setKv(TurboLogger.get("Steer_kV_" + moduleId, DriveConstants.kVSteerDefault));
        if (!steerPIDConfig.closedLoop.flatten().equals(new ClosedLoopConfig().flatten())) steerMotor.configure(steerPIDConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        TurboLogger.log("/Drive/Module" + moduleId + "/SteerAbsPosition", getSteerAbsAngle().in(Degrees));
        TurboLogger.log("/Drive/Module" + moduleId + "/SteerPosition/Actual", getSteerAngle().in(Degrees));
        TurboLogger.log("/Drive/Module" + moduleId + "/SteerVelocity", getSteerVelocity().in(DegreesPerSecond));
        TurboLogger.log("/Drive/Module" + moduleId + "/SteerAcceleration", getSteerAcceleration().in(DegreesPerSecondPerSecond));

        TurboLogger.log("/Drive/Module" + moduleId + "/DrivePosition", getDrivePosition().in(Meters));
        TurboLogger.log("/Drive/Module" + moduleId + "/DriveVelocity/Actual", getDriveVelocity().in(MetersPerSecond));
        TurboLogger.log("/Drive/Module" + moduleId + "/DriveAcceleration", getDriveAcceleration().in(MetersPerSecondPerSecond));

        TurboLogger.log("/Drive/Module" + moduleId + "/DriveVoltage", getSteerVoltage().in(Volts));
        TurboLogger.log("/Drive/Module" + moduleId + "/SteerVoltage", getDriveVoltage().in(Volts));

        TurboLogger.log("/Drive/Module" + moduleId + "/DriveCurrent", getDriveCurrent().in(Amps));
        TurboLogger.log("/Drive/Module" + moduleId + "/SteerCurrent", getSteerCurrent().in(Amps));

        TurboLogger.log("/Drive/Module" + moduleId + "/DriveTemperature", getDriveTemperature().in(Celsius));
        TurboLogger.log("/Drive/Module" + moduleId + "/SteerTemperature", getSteerTemperature().in(Celsius));

        TurboLogger.log("/Drive/Module" + moduleId + "/ModulePosition", getModulePosition());
        TurboLogger.log("/Drive/Module" + moduleId + "/ModuleState", getModuleState());
    }

    @Override
    public void setState(SwerveModuleState state) {
        double driveFFVolts = driveFFController.calculate(state.speedMetersPerSecond);
        double steerFFVolts = steerFFController.calculate((state.angle.getRotations() - getSteerAngle().in(Rotations)) / 0.02);

        driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, driveFFVolts);
        steerController.setReference(state.angle.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0, steerFFVolts);
    }

    @Override
    public void resetPosition(SwerveModulePosition position) {
        driveEncoder.setPosition(position.distanceMeters);
        steerEncoder.setPosition(position.angle.getRotations());
    }

    @Override
    public Angle getSteerAbsAngle() {
        return absEncoder.getAbsolutePosition().getValue();
    }

    @Override
    public Angle getSteerAngle() {
        return Radians.of(steerEncoder.getPosition());
    }

    public AngularVelocity getSteerVelocity() {
        return RadiansPerSecond.of(steerEncoder.getVelocity());
    }

    public Distance getDrivePosition() {
        return Meters.of(driveEncoder.getPosition());
    }

    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(driveEncoder.getVelocity());
    }

    public Voltage getDriveVoltage() {
        return Volts.of(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput());
    }

    public Voltage getSteerVoltage() {
        return Volts.of(steerMotor.getBusVoltage() * steerMotor.getAppliedOutput());
    }

    public Current getDriveCurrent() {
        return Amps.of(driveMotor.getOutputCurrent());
    }

    public Current getSteerCurrent() {
        return Amps.of(steerMotor.getOutputCurrent());
    }

    public Temperature getDriveTemperature() {
        return Celsius.of(driveMotor.getMotorTemperature());
    }

    public Temperature getSteerTemperature() {
        return Celsius.of(steerMotor.getMotorTemperature());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerAngle()));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
    }
}
