
// package frc.robot.subsystems.drive;

// import static edu.wpi.first.units.Units.*;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularAcceleration;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.LinearAcceleration;
// import edu.wpi.first.units.measure.LinearVelocity;
// import edu.wpi.first.units.measure.Voltage;
// import frc.robot.util.AdjustableValues;

// public class ModuleTalonFX extends Module {
//     private int moduleId;

//     // Hardware
//     private TalonFX driveMotor;
//     private TalonFX steerMotor;

//     // Control Modes
//     private PositionVoltage steerControl = new PositionVoltage(0).withSlot(0);
//     private VelocityVoltage driveControl = new VelocityVoltage(0).withSlot(0);

//     /**
//      * Creates a new ModuleIO with TalonFX motors.
//      *
//      * @param moduleId The module id used for logging and getting configs.
//      */
//     public ModuleTalonFX(int moduleId) {
//         this.moduleId = moduleId;

//         driveMotor = new TalonFX((int) DriveConstants.moduleConfigs[moduleId][0]);
//         steerMotor = new TalonFX((int) DriveConstants.moduleConfigs[moduleId][1]);

//         TalonFXConfiguration driveConfig = new TalonFXConfiguration();
//         driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.driveCurrentLimit.in(Amps);
//         driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//         driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveGearRatio;
//         driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//         driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         driveConfig.Slot0.kP = AdjustableValues.getNumber("Drive_kP_" + moduleId);
//         driveConfig.Slot0.kI = AdjustableValues.getNumber("Drive_kI_" + moduleId);
//         driveConfig.Slot0.kD = AdjustableValues.getNumber("Drive_kD_" + moduleId);
//         driveConfig.Slot0.kS = AdjustableValues.getNumber("Drive_kS_" + moduleId);
//         driveConfig.Slot0.kV = AdjustableValues.getNumber("Drive_kV_" + moduleId);
//         driveConfig.Voltage.PeakForwardVoltage = 12;
//         driveConfig.Voltage.PeakReverseVoltage = -12;

//         TalonFXConfiguration steerConfig = new TalonFXConfiguration();
//         steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
//         steerConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.steerCurrentLimit.in(Amps);
//         steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//         steerConfig.Feedback.FeedbackRemoteSensorID = (int) DriveConstants.moduleConfigs[moduleId][2];
//         steerConfig.Feedback.FeedbackRotorOffset = DriveConstants.moduleConfigs[moduleId][3];
//         steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
//         steerConfig.Feedback.SensorToMechanismRatio = DriveConstants.steerGearRatio;
//         steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//         steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
//         steerConfig.Slot0.kP = AdjustableValues.getNumber("Steer_kP_" + moduleId);
//         steerConfig.Slot0.kI = AdjustableValues.getNumber("Steer_kI_" + moduleId);
//         steerConfig.Slot0.kD = AdjustableValues.getNumber("Steer_kD_" + moduleId);
//         steerConfig.Slot0.kS = AdjustableValues.getNumber("Steer_kS_" + moduleId);
//         steerConfig.Slot0.kV = AdjustableValues.getNumber("Steer_kV_" + moduleId);
//         steerConfig.Voltage.PeakForwardVoltage = 12;
//         steerConfig.Voltage.PeakReverseVoltage = -12;

//         driveMotor.getConfigurator().apply(driveConfig);
//         steerMotor.getConfigurator().apply(steerConfig);
//     }

//     @Override
//     public void periodic() {
//         // Updating PID values

//         // What does refresh do vs apply?
//         // driveMotor.getConfigurator().refresh(null);
//         Slot0Configs drivePIDConfig = new Slot0Configs();
//         if (AdjustableValues.hasChanged("Drive_kP_" + moduleId)) drivePIDConfig.kP = AdjustableValues.getNumber("Drive_kP_" + moduleId);
//         if (AdjustableValues.hasChanged("Drive_kI_" + moduleId)) drivePIDConfig.kI = AdjustableValues.getNumber("Drive_kI_" + moduleId);
//         if (AdjustableValues.hasChanged("Drive_kD_" + moduleId)) drivePIDConfig.kD = AdjustableValues.getNumber("Drive_kD_" + moduleId);
//         if (AdjustableValues.hasChanged("Drive_kS_" + moduleId)) drivePIDConfig.kS = AdjustableValues.getNumber("Drive_kS_" + moduleId);
//         if (AdjustableValues.hasChanged("Drive_kV_" + moduleId)) drivePIDConfig.kV = AdjustableValues.getNumber("Drive_kV_" + moduleId);
//         if (!drivePIDConfig.equals(new Slot0Configs())) driveMotor.getConfigurator().refresh(drivePIDConfig);

//         Slot0Configs steerPIDConfig = new Slot0Configs();
//         if (AdjustableValues.hasChanged("Steer_kP_" + moduleId)) steerPIDConfig.kP = AdjustableValues.getNumber("Steer_kP_" + moduleId);
//         if (AdjustableValues.hasChanged("Steer_kI_" + moduleId)) steerPIDConfig.kI = AdjustableValues.getNumber("Steer_kI_" + moduleId);
//         if (AdjustableValues.hasChanged("Steer_kD_" + moduleId)) steerPIDConfig.kD = AdjustableValues.getNumber("Steer_kD_" + moduleId);
//         if (AdjustableValues.hasChanged("Steer_kS_" + moduleId)) steerPIDConfig.kS = AdjustableValues.getNumber("Steer_kS_" + moduleId);
//         if (AdjustableValues.hasChanged("Steer_kV_" + moduleId)) steerPIDConfig.kV = AdjustableValues.getNumber("Steer_kV_" + moduleId);
//         if (!steerPIDConfig.equals(new Slot0Configs())) steerMotor.getConfigurator().refresh(steerPIDConfig);

//         inputs.steerAngle = new Rotation2d(steerMotor.getPosition().getValue());
//         inputs.steerVelocity = steerMotor.getVelocity().getValue();
//         inputs.steerAcceleration = steerMotor.getAcceleration().getValue();

//         inputs.driveDistance = Meters.of(driveMotor.getPosition().getValue().in(Radians) * DriveConstants.wheelRadius.in(Meters));
//         inputs.driveVelocity = MetersPerSecond.of(driveMotor.getVelocity().getValue().in(RadiansPerSecond) * DriveConstants.wheelRadius.in(Meters));
//         inputs.driveAcceleration = MetersPerSecondPerSecond.of(driveMotor.getAcceleration().getValue().in(RadiansPerSecondPerSecond) * DriveConstants.wheelRadius.in(Meters));

//         inputs.driveVoltage = driveMotor.getMotorVoltage().getValue();
//         inputs.steerVoltage = steerMotor.getMotorVoltage().getValue();

//         inputs.driveCurrent = driveMotor.getStatorCurrent().getValue();
//         inputs.steerCurrent = steerMotor.getStatorCurrent().getValue();

//         inputs.driveTemperature = driveMotor.getDeviceTemp().getValue();
//         inputs.steerTemperature = steerMotor.getDeviceTemp().getValue();

//         inputs.modulePosition = new SwerveModulePosition(inputs.driveDistance, inputs.steerAngle);
//         inputs.moduleState = new SwerveModuleState(inputs.driveVelocity, inputs.steerAngle);
//     }

//     @Override
//     public void setState(SwerveModuleState state) {
//         driveMotor.setControl(driveControl.withVelocity(state.speedMetersPerSecond / 2 / Math.PI / DriveConstants.wheelRadius.in(Meters)));
//         steerMotor.setControl(steerControl.withPosition(state.angle.getMeasure()));
//     }

//     @Override
//     public void resetPosition(SwerveModulePosition position) {
//         steerMotor.setPosition(position.angle.getMeasure());
//         driveMotor.setPosition(Radians.of(position.distanceMeters / DriveConstants.wheelRadius.in(Meters)));
//     }

//     @Override
//     public Angle getSteerAbsAngle() {
//         return steerMotor.getAngularPosition();
//     }

//     @Override
//     public Angle getSteerAngle() {
//         return steerMotor.getAngularPosition();
//     }

//     public AngularVelocity getSteerVelocity() {
//         return steerMotor.getAngularVelocity();
//     }

//     public AngularAcceleration getSteerAcceleration() {
//         return steerMotor.getAngularAcceleration();
//     }

//     public Distance getDrivePosition() {
//         return Meters.of(driveMotor.getPosition().in(Radians) * DriveConstants.wheelRadius.in(Meters));
//     }

//     public LinearVelocity getDriveVelocity() {
//         return MetersPerSecond.of(driveMotor.getVelocity().getValue().in(RadiansPerSecond) * DriveConstants.wheelRadius.in(Meters));
//     }

//     public LinearAcceleration getDriveAcceleration() {
//         return MetersPerSecondPerSecond.of(driveMotor.getAcceleration().getValue().in(RadiansPerSecondPerSecond) * DriveConstants.wheelRadius.in(Meters));
//     }

//     public Voltage getDriveVoltage() {
//         return driveMotor.getMotorVoltage().getValue();
//     }

//     public Voltage getSteerVoltage() {
//         return steerMotor.getMotorVoltage().getValue();
//     }

//     public Current getDriveCurrent() {
//         return driveMotor.getStatorCurrent().getValue();
//     }

//     public Current getSteerCurrent() {
//         return steerMotor.getStatorCurrent().getValue();
//     }

//     public SwerveModulePosition getModulePosition() {
//         return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerAngle()));
//     }

//     public SwerveModuleState getModuleState() {
//         return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
//     }
// }
