package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TurboLogger;
import frc.robot.util.TurboLogger2;

public class ModuleSim extends SubsystemBase implements Module {
    private int moduleId;

    private DCMotorSim driveMotor;
    private DCMotorSim steerMotor;

    private PIDController driveController;
    private PIDController steerController;

    private SimpleMotorFeedforward driveFFController;
    private SimpleMotorFeedforward steerFFController;

    /**
     * Creates a simulated Module.
     *
     * @param moduleId The module id used for logging and getting configs.
     */
    public ModuleSim(int moduleId) {
        this.moduleId = moduleId;

        driveFFController = new SimpleMotorFeedforward(
                TurboLogger.get("Drive_kS_" + moduleId, DriveConstants.kSDriveDefault),
                TurboLogger.get("Drive_kV_" + moduleId, DriveConstants.kVDriveDefault), 0);

        steerFFController = new SimpleMotorFeedforward(
                TurboLogger.get("Steer_kS_" + moduleId, DriveConstants.kSSteerDefault),
                TurboLogger.get("Steer_kV_" + moduleId, DriveConstants.kVSteerDefault), 0);

        driveMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1),
                        DriveConstants.driveMOI,
                        DriveConstants.driveGearRatio),
                DCMotor.getKrakenX60(1));

        steerMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DriveConstants.krakenX44,
                        DriveConstants.steerMOI,
                        DriveConstants.steerGearRatio),
                DriveConstants.krakenX44);

        driveController = new PIDController(
                TurboLogger.get("Drive_kP_" + moduleId, DriveConstants.kPDriveDefault),
                TurboLogger.get("Drive_kI_" + moduleId, DriveConstants.kIDriveDefault),
                TurboLogger.get("Drive_kD_" + moduleId, DriveConstants.kDDriveDefault));

        steerController = new PIDController(
                TurboLogger.get("Steer_kP_" + moduleId, DriveConstants.kPSteerDefault),
                TurboLogger.get("Steer_kI_" + moduleId, DriveConstants.kISteerDefault),
                TurboLogger.get("Steer_kD_" + moduleId, DriveConstants.kDSteerDefault));

        steerController.enableContinuousInput(Math.PI, -Math.PI);
    }

    @Override
    public void periodic() {
        if (TurboLogger.hasChanged("Drive_kP_" + moduleId)) driveController.setP(TurboLogger2.getDouble("Drive_kP_" + moduleId));
        if (TurboLogger.hasChanged("Drive_kI_" + moduleId)) driveController.setI(TurboLogger.get("Drive_kI_" + moduleId, DriveConstants.kIDriveDefault));
        if (TurboLogger.hasChanged("Drive_kD_" + moduleId)) driveController.setD(TurboLogger.get("Drive_kD_" + moduleId, DriveConstants.kDDriveDefault));
        if (TurboLogger.hasChanged("Drive_kS_" + moduleId)) driveFFController.setKs(TurboLogger.get("Drive_kS_" + moduleId, DriveConstants.kSDriveDefault));
        if (TurboLogger.hasChanged("Drive_kV_" + moduleId)) driveFFController.setKv(TurboLogger.get("Drive_kV_" + moduleId, DriveConstants.kVDriveDefault));

        if (TurboLogger.hasChanged("Steer_kP_" + moduleId)) steerController.setP(TurboLogger.get("Steer_kP_" + moduleId, DriveConstants.kPSteerDefault));
        if (TurboLogger.hasChanged("Steer_kI_" + moduleId)) steerController.setI(TurboLogger.get("Steer_kI_" + moduleId, DriveConstants.kISteerDefault));
        if (TurboLogger.hasChanged("Steer_kD_" + moduleId)) steerController.setD(TurboLogger.get("Steer_kD_" + moduleId, DriveConstants.kDSteerDefault));
        if (TurboLogger.hasChanged("Steer_kS_" + moduleId)) steerFFController.setKs(TurboLogger.get("Steer_kS_" + moduleId, DriveConstants.kSSteerDefault));
        if (TurboLogger.hasChanged("Steer_kV_" + moduleId)) steerFFController.setKv(TurboLogger.get("Steer_kV_" + moduleId, DriveConstants.kVSteerDefault));

        // Logging the applied PIDFF values
        TurboLogger.log("/Drive/Module" + moduleId + "/PIDS/Drive_kP", driveController.getP());
        TurboLogger.log("/Drive/Module" + moduleId + "/PIDS/Drive_kI", driveController.getI());
        TurboLogger.log("/Drive/Module" + moduleId + "/PIDS/Drive_kD", driveController.getD());
        TurboLogger.log("/Drive/Module" + moduleId + "/PIDS/Drive_kS", driveFFController.getKs());
        TurboLogger.log("/Drive/Module" + moduleId + "/PIDS/Drive_kV", driveFFController.getKv());

        TurboLogger.log("/Drive/Module" + moduleId + "/PIDS/Steer_kP", steerController.getP());
        TurboLogger.log("/Drive/Module" + moduleId + "/PIDS/Steer_kI", steerController.getI());
        TurboLogger.log("/Drive/Module" + moduleId + "/PIDS/Steer_kD", steerController.getD());
        TurboLogger.log("/Drive/Module" + moduleId + "/PIDS/Steer_kS", steerFFController.getKs());
        TurboLogger.log("/Drive/Module" + moduleId + "/PIDS/Steer_kV", steerFFController.getKv());

        // Calculating PIDFF output
        double driveVolts = driveController.calculate(driveMotor.getAngularVelocityRadPerSec()) + driveFFController.calculate(driveController.getSetpoint());
        double steerVolts = steerController.calculate(steerMotor.getAngularPositionRad()) + steerFFController.calculate((steerController.getSetpoint() - steerMotor.getAngularPosition().in(Radians)) / 0.02);

        driveMotor.setInputVoltage(driveVolts);
        steerMotor.setInputVoltage(steerVolts);

        driveMotor.update(0.02);
        steerMotor.update(0.02);

        // Logging
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

        TurboLogger.log("/Drive/Module" + moduleId + "/ModulePosition", getModulePosition());
        TurboLogger.log("/Drive/Module" + moduleId + "/ModuleState/Actual", getModuleState());
    }

    @Override
    public Angle getSteerAbsAngle() {
        return steerMotor.getAngularPosition();
    }

    @Override
    public Angle getSteerAngle() {
        return steerMotor.getAngularPosition();
    }

    @Override
    public AngularVelocity getSteerVelocity() {
        return steerMotor.getAngularVelocity();
    }

    @Override
    public AngularAcceleration getSteerAcceleration() {
        return steerMotor.getAngularAcceleration();
    }

    @Override
    public Distance getDrivePosition() {
        return DriveConstants.wheelRadius.times(driveMotor.getAngularPositionRad());
    }

    @Override
    public LinearVelocity getDriveVelocity() {
        return DriveConstants.wheelRadius.per(Second).times(driveMotor.getAngularVelocityRadPerSec());
    }

    @Override
    public LinearAcceleration getDriveAcceleration() {
        return DriveConstants.wheelRadius.per(Seconds).per(Seconds).times(driveMotor.getAngularAccelerationRadPerSecSq());
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

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerAngle()));
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
    }

    @Override
    public void setState(SwerveModuleState state) {
        TurboLogger.log("/Drive/Module" + moduleId + "/ModuleState/Setpoint", state);

        driveController.setSetpoint(state.speedMetersPerSecond / DriveConstants.wheelRadius.in(Meters));
        steerController.setSetpoint(state.angle.getRadians());
    }

    @Override
    public void resetPosition() {
        driveMotor.setAngle(0);
        steerMotor.setAngle(0);
    }

    @Override
    public void resetPosition(SwerveModulePosition position) {
        driveMotor.setAngle(position.distanceMeters / DriveConstants.wheelRadius.in(Meters));
        steerMotor.setAngle(position.angle.getRadians());
    }
}