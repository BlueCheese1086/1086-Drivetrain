package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.AdjustableValues;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ModuleIOTalonFX implements ModuleIO {
    private int moduleId;

    // Hardware
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANcoder absEncoder;

    private double encoderOffset;
    private ModuleIOInputsAutoLogged inputs;

    /**
     * Creates a new ModuleIO with TalonFX motors.
     * 
     * @param moduleId The module id used for logging and getting configs.
     */
    public ModuleIOTalonFX(int moduleId) {
        this.moduleId = moduleId;

        absEncoder = new CANcoder((int) Constants.DriveConstants.moduleConfigs[moduleId][2]);
        encoderOffset = Constants.DriveConstants.moduleConfigs[moduleId][3];

        driveMotor = new TalonFX((int) Constants.DriveConstants.moduleConfigs[moduleId][0]);
        steerMotor = new TalonFX((int) Constants.DriveConstants.moduleConfigs[moduleId][1]);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.DriveConstants.driveCurrentLimit.in(Amps);
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.Feedback.SensorToMechanismRatio = Constants.DriveConstants.driveGearRatio;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveConfig.Slot0.kP = AdjustableValues.getNumber("Drive_kP_" + moduleId);
        driveConfig.Slot0.kI = AdjustableValues.getNumber("Drive_kI_" + moduleId);
        driveConfig.Slot0.kD = AdjustableValues.getNumber("Drive_kD_" + moduleId);
        driveConfig.Slot0.kS = AdjustableValues.getNumber("Drive_kS_" + moduleId);
        driveConfig.Slot0.kV = AdjustableValues.getNumber("Drive_kV_" + moduleId);
        driveConfig.Slot0.kA = AdjustableValues.getNumber("Drive_kA_" + moduleId);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        steerConfig.CurrentLimits.SupplyCurrentLimit = Constants.DriveConstants.steerCurrentLimit.in(Amps);
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.Feedback.SensorToMechanismRatio = Constants.DriveConstants.steerGearRatio;
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        steerConfig.Slot0.kP = AdjustableValues.getNumber("Steer_kP_" + moduleId);
        steerConfig.Slot0.kI = AdjustableValues.getNumber("Steer_kI_" + moduleId);
        steerConfig.Slot0.kD = AdjustableValues.getNumber("Steer_kD_" + moduleId);
        steerConfig.Slot0.kS = AdjustableValues.getNumber("Steer_kS_" + moduleId);
        steerConfig.Slot0.kV = AdjustableValues.getNumber("Steer_kV_" + moduleId);
        steerConfig.Slot0.kA = AdjustableValues.getNumber("Steer_kA_" + moduleId);
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        driveMotor.getConfigurator().apply(driveConfig);
        steerMotor.getConfigurator().apply(steerConfig);

        steerMotor.setPosition(getAbsoluteAngle().getRotations());

        inputs = new ModuleIOInputsAutoLogged();
    }

    @Override
    public void updateInputs() {
        // Updating PID values

        // What does refresh do vs apply?
        // driveMotor.getConfigurator().refresh(null)
        Slot0Configs drivePIDConfig = new Slot0Configs();
        if (AdjustableValues.hasChanged("Drive_kP_" + moduleId)) drivePIDConfig.kP = AdjustableValues.getNumber("Drive_kP_" + moduleId);
        if (AdjustableValues.hasChanged("Drive_kI_" + moduleId)) drivePIDConfig.kI = AdjustableValues.getNumber("Drive_kI_" + moduleId);
        if (AdjustableValues.hasChanged("Drive_kD_" + moduleId)) drivePIDConfig.kD = AdjustableValues.getNumber("Drive_kD_" + moduleId);
        if (AdjustableValues.hasChanged("Drive_kS_" + moduleId)) drivePIDConfig.kS = AdjustableValues.getNumber("Drive_kS_" + moduleId);
        if (AdjustableValues.hasChanged("Drive_kV_" + moduleId)) drivePIDConfig.kV = AdjustableValues.getNumber("Drive_kV_" + moduleId);
        if (AdjustableValues.hasChanged("Drive_kA_" + moduleId)) drivePIDConfig.kA = AdjustableValues.getNumber("Drive_kA_" + moduleId);
        if (!drivePIDConfig.equals(new Slot0Configs())) driveMotor.getConfigurator().refresh(drivePIDConfig);

        Slot0Configs steerPIDConfig = new Slot0Configs();
        if (AdjustableValues.hasChanged("Steer_kP_" + moduleId)) steerPIDConfig.kP = AdjustableValues.getNumber("Steer_kP_" + moduleId);
        if (AdjustableValues.hasChanged("Steer_kI_" + moduleId)) steerPIDConfig.kI = AdjustableValues.getNumber("Steer_kI_" + moduleId);
        if (AdjustableValues.hasChanged("Steer_kD_" + moduleId)) steerPIDConfig.kD = AdjustableValues.getNumber("Steer_kD_" + moduleId);
        if (AdjustableValues.hasChanged("Steer_kS_" + moduleId)) steerPIDConfig.kS = AdjustableValues.getNumber("Steer_kS_" + moduleId);
        if (AdjustableValues.hasChanged("Steer_kV_" + moduleId)) steerPIDConfig.kV = AdjustableValues.getNumber("Steer_kV_" + moduleId);
        if (AdjustableValues.hasChanged("Steer_kA_" + moduleId)) steerPIDConfig.kA = AdjustableValues.getNumber("Steer_kA_" + moduleId);
        if (!steerPIDConfig.equals(new Slot0Configs())) steerMotor.getConfigurator().refresh(steerPIDConfig);

        inputs.modulePosition = getPosition();
        inputs.moduleState = getState();

        inputs.steerAbsAngle = getAbsoluteAngle();

        inputs.steerAngle = getAngle();
        inputs.steerVelocity = getSteerVelocity();
        inputs.steerAcceleration = getSteerAcceleration();

        inputs.driveDistance = getDistance();
        inputs.driveVelocity = getDriveVelocity();
        inputs.driveAcceleration = getDriveAcceleration();

        inputs.driveVoltage = getDriveVoltage();
        inputs.steerVoltage = getSteerVoltage();

        inputs.driveCurrent = getDriveCurrent();
        inputs.steerCurrent = getSteerCurrent();

        inputs.driveTemperature = getDriveTemperature();
        inputs.steerTemperature = getSteerTemperature();

        Logger.processInputs(String.format("/RealOutputs/Subsystems/Drivetrain/Module%d_TalonFX", moduleId), inputs);
    }

    @Override
    public void setState(SwerveModuleState state) {
        driveMotor.setControl(new VelocityVoltage(RadiansPerSecond.of(state.speedMetersPerSecond / Constants.DriveConstants.wheelRadius.in(Meters))));
        steerMotor.setControl(new PositionVoltage(state.angle.getMeasure()));
    }

    @Override
    public void resetPosition(SwerveModulePosition position) {
        steerMotor.setPosition(position.angle.getMeasure());
        driveMotor.setPosition(Radians.of(position.distanceMeters / Constants.DriveConstants.wheelRadius.in(Meters)));
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    @Override
    public Rotation2d getAbsoluteAngle() {
        return new Rotation2d(absEncoder.getAbsolutePosition().getValue()).minus(Rotation2d.fromRotations(encoderOffset));
    }

    @Override
    public Distance getDistance() {
        return Meters.of(driveMotor.getPosition().getValue().in(Radians) * Constants.DriveConstants.wheelRadius.in(Meters));
    }

    @Override
    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(driveMotor.getVelocity().getValue().in(RadiansPerSecond) * Constants.DriveConstants.wheelRadius.in(Meters));
    }

    @Override
    public LinearAcceleration getDriveAcceleration() {
        return MetersPerSecondPerSecond.of(driveMotor.getAcceleration().getValue().in(RadiansPerSecondPerSecond) * Constants.DriveConstants.wheelRadius.in(Meters));
    }

    @Override
    public Rotation2d getAngle() {
        return new Rotation2d(steerMotor.getPosition().getValue());
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
}