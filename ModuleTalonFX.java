package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TurboLogger;

public class ModuleTalonFX extends SubsystemBase implements Module {
    private int moduleId;

    // Hardware
    private TalonFX driveMotor;
    private TalonFX steerMotor;

    // Control Modes
    private PositionVoltage steerControl = new PositionVoltage(0).withSlot(0);
    private VelocityVoltage driveControl = new VelocityVoltage(0).withSlot(0);

    /**
     * Creates a new ModuleIO with TalonFX motors.
     *
     * @param moduleId The module id used for logging and getting configs.
     */
    public ModuleTalonFX(int moduleId) {
        this.moduleId = moduleId;

        driveMotor = new TalonFX((int) DriveConstants.moduleConfigs[moduleId][0]);
        steerMotor = new TalonFX((int) DriveConstants.moduleConfigs[moduleId][1]);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.driveCurrentLimit.in(Amps);
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveGearRatio;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0.kP = TurboLogger.get("Drive_kP_" + moduleId, DriveConstants.kPDriveDefault);
        driveConfig.Slot0.kI = TurboLogger.get("Drive_kI_" + moduleId, DriveConstants.kIDriveDefault);
        driveConfig.Slot0.kD = TurboLogger.get("Drive_kD_" + moduleId, DriveConstants.kDDriveDefault);
        driveConfig.Slot0.kS = TurboLogger.get("Drive_kS_" + moduleId, DriveConstants.kSDriveDefault);
        driveConfig.Slot0.kV = TurboLogger.get("Drive_kV_" + moduleId, DriveConstants.kVDriveDefault);
        driveConfig.Voltage.PeakForwardVoltage = 12;
        driveConfig.Voltage.PeakReverseVoltage = -12;

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.steerCurrentLimit.in(Amps);
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.Feedback.FeedbackRemoteSensorID = (int) DriveConstants.moduleConfigs[moduleId][2];
        steerConfig.Feedback.FeedbackRotorOffset = DriveConstants.moduleConfigs[moduleId][3];
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        steerConfig.Feedback.SensorToMechanismRatio = DriveConstants.steerGearRatio;
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        steerConfig.Slot0.kP = TurboLogger.get("Steer_kP_" + moduleId, DriveConstants.kPSteerDefault);
        steerConfig.Slot0.kI = TurboLogger.get("Steer_kI_" + moduleId, DriveConstants.kISteerDefault);
        steerConfig.Slot0.kD = TurboLogger.get("Steer_kD_" + moduleId, DriveConstants.kDSteerDefault);
        steerConfig.Slot0.kS = TurboLogger.get("Steer_kS_" + moduleId, DriveConstants.kSSteerDefault);
        steerConfig.Slot0.kV = TurboLogger.get("Steer_kV_" + moduleId, DriveConstants.kVSteerDefault);
        steerConfig.Voltage.PeakForwardVoltage = 12;
        steerConfig.Voltage.PeakReverseVoltage = -12;

        driveMotor.getConfigurator().apply(driveConfig);
        steerMotor.getConfigurator().apply(steerConfig);
    }

    @Override
    public void periodic() {
        // Updating PID values

        // What does refresh do vs apply?
        // driveMotor.getConfigurator().refresh(null);
        Slot0Configs drivePIDConfig = new Slot0Configs();
        if (TurboLogger.hasChanged("Drive_kP_" + moduleId)) drivePIDConfig.kP = TurboLogger.get("Drive_kP_" + moduleId, DriveConstants.kPDriveDefault);
        if (TurboLogger.hasChanged("Drive_kI_" + moduleId)) drivePIDConfig.kI = TurboLogger.get("Drive_kI_" + moduleId, DriveConstants.kIDriveDefault);
        if (TurboLogger.hasChanged("Drive_kD_" + moduleId)) drivePIDConfig.kD = TurboLogger.get("Drive_kD_" + moduleId, DriveConstants.kDDriveDefault);
        if (TurboLogger.hasChanged("Drive_kS_" + moduleId)) drivePIDConfig.kS = TurboLogger.get("Drive_kS_" + moduleId, DriveConstants.kSDriveDefault);
        if (TurboLogger.hasChanged("Drive_kV_" + moduleId)) drivePIDConfig.kV = TurboLogger.get("Drive_kV_" + moduleId, DriveConstants.kVDriveDefault);
        if (!drivePIDConfig.equals(new Slot0Configs())) driveMotor.getConfigurator().refresh(drivePIDConfig);

        Slot0Configs steerPIDConfig = new Slot0Configs();
        if (TurboLogger.hasChanged("Steer_kP_" + moduleId)) steerPIDConfig.kP = TurboLogger.get("Steer_kP_" + moduleId, DriveConstants.kPSteerDefault);
        if (TurboLogger.hasChanged("Steer_kI_" + moduleId)) steerPIDConfig.kI = TurboLogger.get("Steer_kI_" + moduleId, DriveConstants.kISteerDefault);
        if (TurboLogger.hasChanged("Steer_kD_" + moduleId)) steerPIDConfig.kD = TurboLogger.get("Steer_kD_" + moduleId, DriveConstants.kDSteerDefault);
        if (TurboLogger.hasChanged("Steer_kS_" + moduleId)) steerPIDConfig.kS = TurboLogger.get("Steer_kS_" + moduleId, DriveConstants.kSSteerDefault);
        if (TurboLogger.hasChanged("Steer_kV_" + moduleId)) steerPIDConfig.kV = TurboLogger.get("Steer_kV_" + moduleId, DriveConstants.kVSteerDefault);
        if (!steerPIDConfig.equals(new Slot0Configs())) steerMotor.getConfigurator().refresh(steerPIDConfig);

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
        driveMotor.setControl(driveControl.withVelocity(state.speedMetersPerSecond / 2 / Math.PI / DriveConstants.wheelRadius.in(Meters)));
        steerMotor.setControl(steerControl.withPosition(state.angle.getMeasure()));
    }

    @Override
    public void resetPosition() {
        steerMotor.setPosition(0);
        driveMotor.setPosition(0);
    }

    @Override
    public void resetPosition(SwerveModulePosition position) {
        steerMotor.setPosition(position.angle.getMeasure());
        driveMotor.setPosition(Radians.of(position.distanceMeters / DriveConstants.wheelRadius.in(Meters)));
    }

    @Override
    public Angle getSteerAbsAngle() {
        return steerMotor.getPosition().getValue();
    }

    @Override
    public Angle getSteerAngle() {
        return steerMotor.getPosition().getValue();
    }

    @Override
    public AngularVelocity getSteerVelocity() {
        return steerMotor.getVelocity().getValue();
    }

    @Override
    public AngularAcceleration getSteerAcceleration() {
        return steerMotor.getAcceleration().getValue();
    }

    @Override
    public Distance getDrivePosition() {
        return DriveConstants.wheelRadius.times(driveMotor.getPosition().getValue().in(Radians));
    }

    @Override
    public LinearVelocity getDriveVelocity() {
        return DriveConstants.wheelRadius.per(Second).times(driveMotor.getVelocity().getValue().in(RadiansPerSecond));
    }

    @Override
    public LinearAcceleration getDriveAcceleration() {
        return DriveConstants.wheelRadius.per(Second).per(Second).times(driveMotor.getAcceleration().getValue().in(RadiansPerSecondPerSecond));
    }

    @Override
    public Voltage getDriveVoltage() {
        return driveMotor.getMotorVoltage().getValue();
    }

    @Override
    public Voltage getSteerVoltage() {
        return steerMotor.getMotorVoltage().getValue();
    }

    @Override
    public Current getDriveCurrent() {
        return driveMotor.getStatorCurrent().getValue();
    }

    @Override
    public Current getSteerCurrent() {
        return steerMotor.getStatorCurrent().getValue();
    }

    @Override
    public Temperature getDriveTemperature() {
        return driveMotor.getDeviceTemp().getValue();
    }

    @Override
    public Temperature getSteerTemperature() {
        return steerMotor.getDeviceTemp().getValue();
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerAngle()));
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
    }
}