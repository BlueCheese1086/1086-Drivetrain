package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.util.AdjustableValues;
import org.littletonrobotics.junction.Logger;

public class ModuleIOSim implements ModuleIO {
    private int moduleId;

    private DCMotorSim driveMotor;
    private DCMotorSim steerMotor;

    private PIDController driveController;
    private PIDController steerController;

    private SimpleMotorFeedforward driveFFController;
    private SimpleMotorFeedforward steerFFController;

    private SwerveModuleState setpoint = new SwerveModuleState();

    private ModuleIOInputsAutoLogged inputs;

    /**
     * Creates a simulated ModuleIO.
     * 
     * @param moduleId The module id used for logging and getting configs.
     */
    public ModuleIOSim(int moduleId) {
        this.moduleId = moduleId;

        driveFFController = new SimpleMotorFeedforward(AdjustableValues.getNumber("Drive_kS_" + moduleId), AdjustableValues.getNumber("Drive_kV_" + moduleId), AdjustableValues.getNumber("Drive_kA_" + moduleId));
        steerFFController = new SimpleMotorFeedforward(AdjustableValues.getNumber("Steer_kS_" + moduleId), AdjustableValues.getNumber("Steer_kV_" + moduleId), AdjustableValues.getNumber("Steer_kA_" + moduleId));

        driveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), DriveConstants.driveMOI, DriveConstants.driveGearRatio), DCMotor.getKrakenX60(1));
        steerMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DriveConstants.krakenX44, DriveConstants.steerMOI, DriveConstants.steerGearRatio), DriveConstants.krakenX44);

        driveController = new PIDController(AdjustableValues.getNumber("Drive_kP_" + moduleId), AdjustableValues.getNumber("Drive_kI_" + moduleId), AdjustableValues.getNumber("Drive_kD_" + moduleId));
        steerController = new PIDController(AdjustableValues.getNumber("Steer_kP_" + moduleId), AdjustableValues.getNumber("Steer_kI_" + moduleId), AdjustableValues.getNumber("Steer_kD_" + moduleId));

        steerController.enableContinuousInput(Math.PI, -Math.PI);

        inputs = new ModuleIOInputsAutoLogged();
    }

    @Override
    public void updateInputs() {
        if (AdjustableValues.hasChanged("Drive_kP_" + moduleId)) driveController.setP(AdjustableValues.getNumber("Drive_kP_" + moduleId));
        if (AdjustableValues.hasChanged("Drive_kI_" + moduleId)) driveController.setI(AdjustableValues.getNumber("Drive_kI_" + moduleId));
        if (AdjustableValues.hasChanged("Drive_kD_" + moduleId)) driveController.setD(AdjustableValues.getNumber("Drive_kD_" + moduleId));

        if (AdjustableValues.hasChanged("Steer_kP_" + moduleId)) steerController.setP(AdjustableValues.getNumber("Steer_kP_" + moduleId));
        if (AdjustableValues.hasChanged("Steer_kI_" + moduleId)) steerController.setI(AdjustableValues.getNumber("Steer_kI_" + moduleId));
        if (AdjustableValues.hasChanged("Steer_kD_" + moduleId)) steerController.setD(AdjustableValues.getNumber("Steer_kD_" + moduleId));

        if (AdjustableValues.hasChanged("Drive_kS_" + moduleId)) driveFFController.setKs(AdjustableValues.getNumber("Drive_kS_" + moduleId));
        if (AdjustableValues.hasChanged("Drive_kV_" + moduleId)) driveFFController.setKv(AdjustableValues.getNumber("Drive_kV_" + moduleId));
        if (AdjustableValues.hasChanged("Drive_kA_" + moduleId)) driveFFController.setKa(AdjustableValues.getNumber("Drive_kA_" + moduleId));

        if (AdjustableValues.hasChanged("Steer_kS_" + moduleId)) steerFFController.setKs(AdjustableValues.getNumber("Steer_kS_" + moduleId));
        if (AdjustableValues.hasChanged("Steer_kV_" + moduleId)) steerFFController.setKv(AdjustableValues.getNumber("Steer_kV_" + moduleId));
        if (AdjustableValues.hasChanged("Steer_kA_" + moduleId)) steerFFController.setKa(AdjustableValues.getNumber("Steer_kA_" + moduleId));

        double driveVolts = MathUtil.clamp(driveController.calculate(getDriveVelocity().in(MetersPerSecond)) + driveMotor.getInputVoltage() + driveFFController.calculate(setpoint.speedMetersPerSecond), -12, 12);
        double steerVolts = MathUtil.clamp(steerController.calculate(getAngle().getRadians()) + steerFFController.calculate((setpoint.angle.minus(getAngle()).getRadians()) / 0.02), -12, 12);

        driveMotor.setInputVoltage(driveVolts);
        steerMotor.setInputVoltage(steerVolts);

        driveMotor.update(0.02);
        steerMotor.update(0.02);

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

        Logger.processInputs(String.format("/RealOutputs/Subsystems/Drivetrain/Module%d_Sim", moduleId), inputs);
    }

    @Override
    public void setState(SwerveModuleState state) {
        setpoint = state;

        driveController.setSetpoint(state.speedMetersPerSecond);
        steerController.setSetpoint(state.angle.getRadians());
    }

    @Override
    public void resetPosition(SwerveModulePosition position) {
        driveMotor.setAngle(position.distanceMeters / DriveConstants.wheelRadius.in(Meters));
        steerMotor.setAngle(position.angle.getRadians());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAbsoluteAngle());
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getAbsoluteAngle());
    }

    @Override
    public Rotation2d getAbsoluteAngle() {
        return new Rotation2d(steerMotor.getAngularPosition());
    }

    @Override
    public Rotation2d getAngle() {
        return new Rotation2d(steerMotor.getAngularPositionRad());
    }

    @Override
    public AngularVelocity getSteerVelocity() {
        return RadiansPerSecond.of(steerMotor.getAngularVelocityRadPerSec());
    }

    @Override
    public AngularAcceleration getSteerAcceleration() {
        return RadiansPerSecondPerSecond.of(steerMotor.getAngularAccelerationRadPerSecSq());
    }

    @Override
    public Distance getDistance() {
        return Meters.of(driveMotor.getAngularPositionRad() * DriveConstants.driveGearRatio * DriveConstants.wheelRadius.in(Meters));
    }

    @Override
    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(driveMotor.getAngularVelocityRadPerSec() * DriveConstants.driveGearRatio * DriveConstants.wheelRadius.in(Meters));
    }

    @Override
    public LinearAcceleration getDriveAcceleration() {
        return MetersPerSecondPerSecond.of(driveMotor.getAngularAccelerationRadPerSecSq() * DriveConstants.driveGearRatio * DriveConstants.wheelRadius.in(Meters));
    }

    @Override
    public Voltage getDriveVoltage() {
        return Volts.of(driveMotor.getInputVoltage());
    }

    @Override
    public Voltage getSteerVoltage() {
        return Volts.of(steerMotor.getInputVoltage());
    }

    @Override
    public Current getDriveCurrent() {
        return Amps.of(driveMotor.getCurrentDrawAmps());
    }

    @Override
    public Current getSteerCurrent() {
        return Amps.of(steerMotor.getCurrentDrawAmps());
    }

    @Override
    public Temperature getDriveTemperature() {
        return Celsius.zero();
    }

    @Override
    public Temperature getSteerTemperature() {
        return Celsius.zero();
    }
}