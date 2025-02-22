package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

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
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class ModuleIOTalonFX implements ModuleIO {
    private int moduleId;

    private TalonFX driveMotor;
    private TalonFX steerMotor;

    private CANcoder absEncoder;
    private double encoderOffset;

    private ModuleIOInputsAutoLogged inputs;
    
    public ModuleIOTalonFX(int moduleId) {
        this.moduleId = moduleId;

        absEncoder = new CANcoder((int) DriveConstants.moduleConfigs[moduleId][2]);
        encoderOffset = DriveConstants.moduleConfigs[moduleId][3];

        driveMotor = new TalonFX((int) DriveConstants.moduleConfigs[moduleId][0]);
        steerMotor = new TalonFX((int) DriveConstants.moduleConfigs[moduleId][1]);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        
        driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.driveCurrentLimit.in(Amps);
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.Feedback.FeedbackRotorOffset = encoderOffset;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveConfig.Slot0.kP = DriveConstants.kPDriveReal;
        driveConfig.Slot0.kI = DriveConstants.kIDriveReal;
        driveConfig.Slot0.kD = DriveConstants.kDDriveReal;

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        
        steerConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.steerCurrentLimit.in(Amps);
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.Feedback.FeedbackRotorOffset = encoderOffset;
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        steerConfig.Slot0.kP = DriveConstants.kPSteerReal;
        steerConfig.Slot0.kI = DriveConstants.kISteerReal;
        steerConfig.Slot0.kD = DriveConstants.kDSteerReal;
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        driveMotor.getConfigurator().apply(driveConfig);
        steerMotor.getConfigurator().apply(steerConfig);

        steerMotor.setPosition(getAbsoluteAngle().getRotations());

        inputs = new ModuleIOInputsAutoLogged();
    }

    @Override
    public void updateInputs() {
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

        Logger.processInputs(String.format("Module%d_TalonFX", moduleId), inputs);
    }

    @Override
    public void setState(SwerveModuleState state) {
        driveMotor.setControl(new VelocityVoltage(state.speedMetersPerSecond / DriveConstants.metersPerRotation));
        steerMotor.setControl(new PositionVoltage(state.angle.getMeasure()));
    }

    @Override
    public void resetPosition(SwerveModulePosition position) {
        steerMotor.setPosition(position.angle.getMeasure());
        driveMotor.setPosition(position.distanceMeters / DriveConstants.metersPerRotation);
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
        return new Rotation2d(absEncoder.getAbsolutePosition().getValue()).minus(new Rotation2d(encoderOffset));
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
    public Distance getDistance() {
        return Meters.of(driveMotor.getPosition().getValueAsDouble() * DriveConstants.metersPerRotation);
    }

    @Override
    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(driveMotor.getVelocity().getValueAsDouble() * DriveConstants.metersPerRotation);
    }

    @Override
    public LinearAcceleration getDriveAcceleration() {
        return MetersPerSecondPerSecond.of(driveMotor.getAcceleration().getValueAsDouble() * DriveConstants.metersPerRotation);
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