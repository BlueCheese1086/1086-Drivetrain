package frc.robot.subsystems.drivetrain;

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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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

public class ModuleIOSparkMax implements ModuleIO {
    private int moduleId;

    private SparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkClosedLoopController driveController;

    private SparkMax steerMotor;
    private RelativeEncoder steerEncoder;
    private SparkClosedLoopController steerController;

    private CANcoder absEncoder;
    private double encoderOffset;

    private ModuleIOInputsAutoLogged inputs;

    public ModuleIOSparkMax(int moduleId) {
        this.moduleId = moduleId;

        absEncoder = new CANcoder((int) DriveConstants.moduleConfigs[moduleId][2]);
        encoderOffset = DriveConstants.moduleConfigs[moduleId][3];

        driveMotor = new SparkMax((int) DriveConstants.moduleConfigs[moduleId][0], MotorType.kBrushless);
        steerMotor = new SparkMax((int) DriveConstants.moduleConfigs[moduleId][1], MotorType.kBrushless);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.closedLoop.p(DriveConstants.kPDriveReal, ClosedLoopSlot.kSlot0);
        driveConfig.closedLoop.i(DriveConstants.kIDriveReal, ClosedLoopSlot.kSlot0);
        driveConfig.closedLoop.d(DriveConstants.kDDriveReal, ClosedLoopSlot.kSlot0);
        driveConfig.encoder.positionConversionFactor(DriveConstants.metersPerRotation);
        driveConfig.encoder.velocityConversionFactor(DriveConstants.metersPerRotation / 60);
        driveConfig.inverted(false);
        driveConfig.idleMode(IdleMode.kCoast);
        driveConfig.smartCurrentLimit((int) DriveConstants.driveCurrentLimit.in(Amps));

        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.closedLoop.p(DriveConstants.kPSteerReal, ClosedLoopSlot.kSlot0);
        steerConfig.closedLoop.i(DriveConstants.kISteerReal, ClosedLoopSlot.kSlot0);
        steerConfig.closedLoop.d(DriveConstants.kDSteerReal, ClosedLoopSlot.kSlot0);
        steerConfig.closedLoop.positionWrappingEnabled(false);
        // steerConfig.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);
        steerConfig.encoder.positionConversionFactor(1.0 / DriveConstants.steerGearRatio);
        steerConfig.encoder.velocityConversionFactor(1.0 / DriveConstants.steerGearRatio);
        steerConfig.inverted(false);
        steerConfig.idleMode(IdleMode.kCoast);
        steerConfig.smartCurrentLimit((int) DriveConstants.steerCurrentLimit.in(Amps));

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        steerEncoder.setPosition(getAbsoluteAngle().getRotations());

        driveController = driveMotor.getClosedLoopController();
        steerController = steerMotor.getClosedLoopController();

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

        Logger.processInputs(String.format("Module%d_SparkMax", moduleId), inputs);
    }

    @Override
    public void setState(SwerveModuleState state) {
        driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0);
        steerController.setReference(state.angle.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
    }

    @Override
    public void resetPosition(SwerveModulePosition position) {
        driveEncoder.setPosition(position.distanceMeters);
        steerEncoder.setPosition(position.angle.getRotations());
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
        return Rotation2d.fromRotations(steerEncoder.getPosition());
    }

    @Override
    public AngularVelocity getSteerVelocity() {
        return Rotations.per(Minute).of(steerEncoder.getVelocity());
    }

    @Override
    public AngularAcceleration getSteerAcceleration() {
        return getSteerVelocity().div(Seconds.of(0.02));
    }

    @Override
    public Distance getDistance() {
        return Meters.of(driveEncoder.getPosition());
    }

    @Override
    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(driveEncoder.getVelocity());
    }

    @Override
    public LinearAcceleration getDriveAcceleration() {
        return getDriveVelocity().div(Seconds.of(0.02));
    }

    @Override
    public Voltage getDriveVoltage() {
        return Volts.of(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
    }

    @Override
    public Voltage getSteerVoltage() {
        return Volts.of(steerMotor.getAppliedOutput() * steerMotor.getBusVoltage());
    }

    @Override
    public Current getDriveCurrent() {
        return Amps.of(driveMotor.getOutputCurrent());
    }

    @Override
    public Current getSteerCurrent() {
        return Amps.of(steerMotor.getOutputCurrent());
    }

    @Override
    public Temperature getDriveTemperature() {
        return Celsius.of(driveMotor.getMotorTemperature());
    }

    @Override
    public Temperature getSteerTemperature() {
        return Celsius.of(steerMotor.getMotorTemperature());
    }
}