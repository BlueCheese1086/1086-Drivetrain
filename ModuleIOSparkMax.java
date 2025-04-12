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
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.AdjustableValues;

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

    private SimpleMotorFeedforward driveFFController;
    private SimpleMotorFeedforward steerFFController;

    private ModuleIOInputs lastInputs;

    /**
     * Creates a new ModuleIO with SparkMAX motors.
     * 
     * @param moduleId The module id used for logging and getting configs.
     */
    public ModuleIOSparkMax(int moduleId) {
        this.moduleId = moduleId;

        absEncoder = new CANcoder((int) DriveConstants.moduleConfigs[moduleId][2]);
        encoderOffset = DriveConstants.moduleConfigs[moduleId][3];

        driveMotor = new SparkMax((int) DriveConstants.moduleConfigs[moduleId][0], MotorType.kBrushless);
        steerMotor = new SparkMax((int) DriveConstants.moduleConfigs[moduleId][1], MotorType.kBrushless);

        driveFFController = new SimpleMotorFeedforward(AdjustableValues.getNumber("Drive_kS_" + moduleId), AdjustableValues.getNumber("Drive_kV_" + moduleId), 0, 0.02);
        steerFFController = new SimpleMotorFeedforward(AdjustableValues.getNumber("Steer_kS_" + moduleId), AdjustableValues.getNumber("Steer_kV_" + moduleId), 0, 0.02);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.closedLoop.p(AdjustableValues.getNumber("Drive_kP_" + moduleId), ClosedLoopSlot.kSlot0);
        driveConfig.closedLoop.i(AdjustableValues.getNumber("Drive_kI_" + moduleId), ClosedLoopSlot.kSlot0);
        driveConfig.closedLoop.d(AdjustableValues.getNumber("Drive_kD_" + moduleId), ClosedLoopSlot.kSlot0);
        driveConfig.encoder.positionConversionFactor(DriveConstants.wheelRadius.in(Meters) / DriveConstants.driveGearRatio);
        driveConfig.encoder.velocityConversionFactor(DriveConstants.wheelRadius.in(Meters) / DriveConstants.driveGearRatio / 60);
        driveConfig.inverted(false);
        driveConfig.idleMode(IdleMode.kCoast);
        driveConfig.smartCurrentLimit((int) DriveConstants.driveCurrentLimit.in(Amps));

        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.closedLoop.p(AdjustableValues.getNumber("Steer_kP_" + moduleId), ClosedLoopSlot.kSlot0);
        steerConfig.closedLoop.i(AdjustableValues.getNumber("Steer_kI_" + moduleId), ClosedLoopSlot.kSlot0);
        steerConfig.closedLoop.d(AdjustableValues.getNumber("Steer_kD_" + moduleId), ClosedLoopSlot.kSlot0);
        steerConfig.closedLoop.positionWrappingEnabled(true);
        steerConfig.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);
        steerConfig.encoder.positionConversionFactor(1.0 / DriveConstants.steerGearRatio);
        steerConfig.encoder.velocityConversionFactor(1.0 / DriveConstants.steerGearRatio);
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

        lastInputs = new ModuleIOInputs();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        SparkMaxConfig drivePIDConfig = new SparkMaxConfig();
        if (AdjustableValues.hasChanged("Drive_kP_" + moduleId)) drivePIDConfig.closedLoop.p(AdjustableValues.getNumber("Drive_kP_" + moduleId));
        if (AdjustableValues.hasChanged("Drive_kI_" + moduleId)) drivePIDConfig.closedLoop.i(AdjustableValues.getNumber("Drive_kI_" + moduleId));
        if (AdjustableValues.hasChanged("Drive_kD_" + moduleId)) drivePIDConfig.closedLoop.d(AdjustableValues.getNumber("Drive_kD_" + moduleId));
        if (AdjustableValues.hasChanged("Drive_kS_" + moduleId)) driveFFController.setKs(AdjustableValues.getNumber("Drive_kS_" + moduleId));
        if (AdjustableValues.hasChanged("Drive_kV_" + moduleId)) driveFFController.setKv(AdjustableValues.getNumber("Drive_kV_" + moduleId));
        if (!drivePIDConfig.closedLoop.flatten().equals(new ClosedLoopConfig().flatten())) driveMotor.configure(drivePIDConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig steerPIDConfig = new SparkMaxConfig();
        if (AdjustableValues.hasChanged("Steer_kP_" + moduleId)) steerPIDConfig.closedLoop.p(AdjustableValues.getNumber("Steer_kP_" + moduleId));
        if (AdjustableValues.hasChanged("Steer_kI_" + moduleId)) steerPIDConfig.closedLoop.i(AdjustableValues.getNumber("Steer_kI_" + moduleId));
        if (AdjustableValues.hasChanged("Steer_kD_" + moduleId)) steerPIDConfig.closedLoop.d(AdjustableValues.getNumber("Steer_kD_" + moduleId));
        if (AdjustableValues.hasChanged("Steer_kS_" + moduleId)) steerFFController.setKs(AdjustableValues.getNumber("Steer_kS_" + moduleId));
        if (AdjustableValues.hasChanged("Steer_kV_" + moduleId)) steerFFController.setKv(AdjustableValues.getNumber("Steer_kV_" + moduleId));
        if (!steerPIDConfig.closedLoop.flatten().equals(new ClosedLoopConfig().flatten())) steerMotor.configure(steerPIDConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        inputs.steerAbsAngle = new Rotation2d(absEncoder.getAbsolutePosition().getValue()).minus(Rotation2d.fromRotations(encoderOffset));

        inputs.steerAngle = Rotation2d.fromRotations(steerEncoder.getPosition());
        inputs.steerVelocity = RPM.of(steerEncoder.getVelocity());
        inputs.steerAcceleration = inputs.steerVelocity.minus(lastInputs.steerVelocity).div(Seconds.of(0.02));

        inputs.driveDistance = Meters.of(driveEncoder.getPosition());
        inputs.driveVelocity = MetersPerSecond.of(driveEncoder.getVelocity());
        inputs.driveAcceleration = inputs.driveVelocity.minus(lastInputs.driveVelocity).div(Seconds.of(0.02));

        inputs.driveVoltage = Volts.of(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
        inputs.steerVoltage = Volts.of(steerMotor.getAppliedOutput() * steerMotor.getBusVoltage());

        inputs.driveCurrent = Amps.of(driveMotor.getOutputCurrent());
        inputs.steerCurrent = Amps.of(steerMotor.getOutputCurrent());

        inputs.driveTemperature = Celsius.of(driveMotor.getMotorTemperature());
        inputs.steerTemperature = Celsius.of(steerMotor.getMotorTemperature());

        inputs.modulePosition = new SwerveModulePosition(inputs.driveDistance, inputs.steerAngle);
        inputs.moduleState = new SwerveModuleState(inputs.driveVelocity, inputs.steerAngle);

        lastInputs = inputs;
    }

    @Override
    public void setState(SwerveModuleState state) {
        double driveFFVolts = driveFFController.calculate(state.speedMetersPerSecond);
        double steerFFVolts = steerFFController.calculate(state.angle.minus(lastInputs.steerAngle).getRotations() / 0.02);

        driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, driveFFVolts);
        steerController.setReference(state.angle.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0, steerFFVolts);
    }

    @Override
    public void resetPosition(SwerveModulePosition position) {
        driveEncoder.setPosition(position.distanceMeters);
        steerEncoder.setPosition(position.angle.getRotations());
    }
}