package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.AdjustableValues;

public class ModuleIOTalonFX implements ModuleIO {
    private int moduleId;

    // Hardware
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANcoder absEncoder;

    // Control Modes
    private PositionVoltage steerControl = new PositionVoltage(0).withSlot(0);
    private VelocityVoltage driveControl = new VelocityVoltage(0).withSlot(0);

    private double encoderOffset;

    /**
     * Creates a new ModuleIO with TalonFX motors.
     * 
     * @param moduleId The module id used for logging and getting configs.
     */
    public ModuleIOTalonFX(int moduleId) {
        this.moduleId = moduleId;

        absEncoder = new CANcoder((int) DriveConstants.moduleConfigs[moduleId][2]);
        encoderOffset = DriveConstants.moduleConfigs[moduleId][3];

        driveMotor = new TalonFX((int) DriveConstants.moduleConfigs[moduleId][0]);
        steerMotor = new TalonFX((int) DriveConstants.moduleConfigs[moduleId][1]);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.driveCurrentLimit;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveGearRatio;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0.kP = AdjustableValues.getNumber("Drive_kP_" + moduleId);
        driveConfig.Slot0.kI = AdjustableValues.getNumber("Drive_kI_" + moduleId);
        driveConfig.Slot0.kD = AdjustableValues.getNumber("Drive_kD_" + moduleId);
        driveConfig.Slot0.kS = AdjustableValues.getNumber("Drive_kS_" + moduleId);
        driveConfig.Slot0.kV = AdjustableValues.getNumber("Drive_kV_" + moduleId);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        steerConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.steerCurrentLimit;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.Feedback.SensorToMechanismRatio = DriveConstants.steerGearRatio;
        // See if this works
        // steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // steerConfig.Feedback.FeedbackRemoteSensorID = (int) DriveConstants.moduleConfigs[moduleId][2];
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        steerConfig.Slot0.kP = AdjustableValues.getNumber("Steer_kP_" + moduleId);
        steerConfig.Slot0.kI = AdjustableValues.getNumber("Steer_kI_" + moduleId);
        steerConfig.Slot0.kD = AdjustableValues.getNumber("Steer_kD_" + moduleId);
        steerConfig.Slot0.kS = AdjustableValues.getNumber("Steer_kS_" + moduleId);
        steerConfig.Slot0.kV = AdjustableValues.getNumber("Steer_kV_" + moduleId);
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        driveMotor.getConfigurator().apply(driveConfig);
        steerMotor.getConfigurator().apply(steerConfig);

        steerMotor.setPosition(absEncoder.getAbsolutePosition().getValueAsDouble() - encoderOffset);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Updating PID values

        // What does refresh do vs apply?
        // driveMotor.getConfigurator().refresh(null);
        Slot0Configs drivePIDConfig = new Slot0Configs();
        if (AdjustableValues.hasChanged("Drive_kP_" + moduleId)) drivePIDConfig.kP = AdjustableValues.getNumber("Drive_kP_" + moduleId);
        if (AdjustableValues.hasChanged("Drive_kI_" + moduleId)) drivePIDConfig.kI = AdjustableValues.getNumber("Drive_kI_" + moduleId);
        if (AdjustableValues.hasChanged("Drive_kD_" + moduleId)) drivePIDConfig.kD = AdjustableValues.getNumber("Drive_kD_" + moduleId);
        if (AdjustableValues.hasChanged("Drive_kS_" + moduleId)) drivePIDConfig.kS = AdjustableValues.getNumber("Drive_kS_" + moduleId);
        if (AdjustableValues.hasChanged("Drive_kV_" + moduleId)) drivePIDConfig.kV = AdjustableValues.getNumber("Drive_kV_" + moduleId);
        if (!drivePIDConfig.equals(new Slot0Configs())) driveMotor.getConfigurator().refresh(drivePIDConfig);

        Slot0Configs steerPIDConfig = new Slot0Configs();
        if (AdjustableValues.hasChanged("Steer_kP_" + moduleId)) steerPIDConfig.kP = AdjustableValues.getNumber("Steer_kP_" + moduleId);
        if (AdjustableValues.hasChanged("Steer_kI_" + moduleId)) steerPIDConfig.kI = AdjustableValues.getNumber("Steer_kI_" + moduleId);
        if (AdjustableValues.hasChanged("Steer_kD_" + moduleId)) steerPIDConfig.kD = AdjustableValues.getNumber("Steer_kD_" + moduleId);
        if (AdjustableValues.hasChanged("Steer_kS_" + moduleId)) steerPIDConfig.kS = AdjustableValues.getNumber("Steer_kS_" + moduleId);
        if (AdjustableValues.hasChanged("Steer_kV_" + moduleId)) steerPIDConfig.kV = AdjustableValues.getNumber("Steer_kV_" + moduleId);
        if (!steerPIDConfig.equals(new Slot0Configs())) steerMotor.getConfigurator().refresh(steerPIDConfig);

        inputs.steerAbsAngle = new Rotation2d(absEncoder.getAbsolutePosition().getValue()).minus(Rotation2d.fromRotations(encoderOffset));

        inputs.steerAngle = new Rotation2d(steerMotor.getPosition().getValue());
        inputs.steerVelocity = steerMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
        inputs.steerAcceleration = steerMotor.getAcceleration().getValueAsDouble() * 2 * Math.PI;

        inputs.driveDistance = driveMotor.getPosition().getValueAsDouble() * 2 * Math.PI * DriveConstants.wheelRadius;
        inputs.driveVelocity = driveMotor.getVelocity().getValueAsDouble() * 2 * Math.PI * DriveConstants.wheelRadius;
        inputs.driveAcceleration = driveMotor.getAcceleration().getValueAsDouble() * 2 * Math.PI * DriveConstants.wheelRadius;

        inputs.driveVoltage = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.steerVoltage = steerMotor.getMotorVoltage().getValueAsDouble();

        inputs.driveCurrent = driveMotor.getStatorCurrent().getValueAsDouble();
        inputs.steerCurrent = steerMotor.getStatorCurrent().getValueAsDouble();

        inputs.driveTemperature = driveMotor.getDeviceTemp().getValueAsDouble();
        inputs.steerTemperature = steerMotor.getDeviceTemp().getValueAsDouble();

        inputs.modulePosition = new SwerveModulePosition(inputs.driveDistance, inputs.steerAngle);
        inputs.moduleState = new SwerveModuleState(inputs.driveVelocity, inputs.steerAngle);
    }

    @Override
    public void setState(SwerveModuleState state) {
        driveMotor.setControl(driveControl.withVelocity(state.speedMetersPerSecond / 2 / Math.PI / DriveConstants.wheelRadius));
        steerMotor.setControl(steerControl.withPosition(state.angle.getMeasure()));
    }

    @Override
    public void resetPosition(SwerveModulePosition position) {
        steerMotor.setPosition(position.angle.getMeasure());
        driveMotor.setPosition(position.distanceMeters / 2 / Math.PI / DriveConstants.wheelRadius);
    }
}