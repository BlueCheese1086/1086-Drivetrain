
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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.TurboLogger;

import org.littletonrobotics.junction.Logger;

public class ModuleSim extends Module {
    private int moduleId;

    private DCMotorSim driveMotor;
    private DCMotorSim steerMotor;

    private PIDController driveController;
    private PIDController steerController;

    private SimpleMotorFeedforward driveFFController;
    private SimpleMotorFeedforward steerFFController;

    /**
     * Creates a simulated ModuleIO.
     *
     * @param moduleId The module id used for logging and getting configs.
     */
    public ModuleSim(int moduleId) {
        this.moduleId = moduleId;

        driveFFController = new SimpleMotorFeedforward(TurboLogger.get("Drive_kS_" + moduleId, DriveConstants.kSDriveDefault), TurboLogger.get("Drive_kV_" + moduleId, DriveConstants.kVDriveDefault), 0);
        steerFFController = new SimpleMotorFeedforward(TurboLogger.get("Steer_kS_" + moduleId, DriveConstants.kSSteerDefault), TurboLogger.get("Steer_kV_" + moduleId, DriveConstants.kVSteerDefault), 0);

        driveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), DriveConstants.driveMOI, DriveConstants.driveGearRatio), DCMotor.getKrakenX60(1));
        steerMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DriveConstants.krakenX44, DriveConstants.steerMOI, DriveConstants.steerGearRatio), DriveConstants.krakenX44);

        driveController = new PIDController(TurboLogger.get("Drive_kP_" + moduleId, DriveConstants.kPDriveDefault), TurboLogger.get("Drive_kI_" + moduleId, DriveConstants.kIDriveDefault), TurboLogger.get("Drive_kD_" + moduleId, DriveConstants.kDDriveDefault));
        steerController = new PIDController(TurboLogger.get("Steer_kP_" + moduleId, DriveConstants.kPSteerDefault), TurboLogger.get("Steer_kI_" + moduleId, DriveConstants.kISteerDefault), TurboLogger.get("Steer_kD_" + moduleId, DriveConstants.kDSteerDefault));

        steerController.enableContinuousInput(Math.PI, -Math.PI);
    }

    @Override
    public void periodic() {
        if (TurboLogger.hasChanged("Drive_kP_" + moduleId)) driveController.setP(TurboLogger.get("Drive_kP_" + moduleId, DriveConstants.kPDriveDefault));
        if (TurboLogger.hasChanged("Drive_kI_" + moduleId)) driveController.setI(TurboLogger.get("Drive_kI_" + moduleId, DriveConstants.kIDriveDefault));
        if (TurboLogger.hasChanged("Drive_kD_" + moduleId)) driveController.setD(TurboLogger.get("Drive_kD_" + moduleId, DriveConstants.kDDriveDefault));
        if (TurboLogger.hasChanged("Drive_kS_" + moduleId)) driveFFController.setKs(TurboLogger.get("Drive_kS_" + moduleId, DriveConstants.kSDriveDefault));
        if (TurboLogger.hasChanged("Drive_kV_" + moduleId)) driveFFController.setKv(TurboLogger.get("Drive_kV_" + moduleId, DriveConstants.kVDriveDefault));

        if (TurboLogger.hasChanged("Steer_kP_" + moduleId)) steerController.setP(TurboLogger.get("Steer_kP_" + moduleId, DriveConstants.kPSteerDefault));
        if (TurboLogger.hasChanged("Steer_kI_" + moduleId)) steerController.setI(TurboLogger.get("Steer_kI_" + moduleId, DriveConstants.kISteerDefault));
        if (TurboLogger.hasChanged("Steer_kD_" + moduleId)) steerController.setD(TurboLogger.get("Steer_kD_" + moduleId, DriveConstants.kDSteerDefault));
        if (TurboLogger.hasChanged("Steer_kS_" + moduleId)) steerFFController.setKs(TurboLogger.get("Steer_kS_" + moduleId, DriveConstants.kSSteerDefault));
        if (TurboLogger.hasChanged("Steer_kV_" + moduleId)) steerFFController.setKv(TurboLogger.get("Steer_kV_" + moduleId, DriveConstants.kVSteerDefault));

        Logger.recordOutput("/Drive/Module" + moduleId + "/PIDS/Drive_kP", driveController.getP());
        Logger.recordOutput("/Drive/Module" + moduleId + "/PIDS/Drive_kI", driveController.getI());
        Logger.recordOutput("/Drive/Module" + moduleId + "/PIDS/Drive_kD", driveController.getD());
        Logger.recordOutput("/Drive/Module" + moduleId + "/PIDS/Drive_kS", driveFFController.getKs());
        Logger.recordOutput("/Drive/Module" + moduleId + "/PIDS/Drive_kV", driveFFController.getKv());

        Logger.recordOutput("/Drive/Module" + moduleId + "/PIDS/Steer_kP", steerController.getP());
        Logger.recordOutput("/Drive/Module" + moduleId + "/PIDS/Steer_kI", steerController.getI());
        Logger.recordOutput("/Drive/Module" + moduleId + "/PIDS/Steer_kD", steerController.getD());
        Logger.recordOutput("/Drive/Module" + moduleId + "/PIDS/Steer_kS", steerFFController.getKs());
        Logger.recordOutput("/Drive/Module" + moduleId + "/PIDS/Steer_kV", steerFFController.getKv());

        double driveVolts = driveController.calculate(driveMotor.getAngularVelocityRadPerSec()) + driveFFController.calculate(driveController.getSetpoint());
        double steerVolts = steerController.calculate(steerMotor.getAngularPositionRad()) + steerFFController.calculate((steerController.getSetpoint() - steerMotor.getAngularPosition().in(Radians)) / 0.02);

        driveMotor.setInputVoltage(driveVolts);
        steerMotor.setInputVoltage(steerVolts);

        driveMotor.update(0.02);
        steerMotor.update(0.02);

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

        TurboLogger.log("/Drive/Module" + moduleId + "/DriveCurrent", getModulePosition());
        TurboLogger.log("/Drive/Module" + moduleId + "/DriveCurrent", getModuleState());
    }

    @Override
    public void setState(SwerveModuleState state) {
        driveController.setSetpoint(state.speedMetersPerSecond / DriveConstants.wheelRadius.in(Meters));
        steerController.setSetpoint(state.angle.getRadians());
        Logger.recordOutput(getSubsystem(), Degrees.of(1));
    }

    @Override
    public void resetPosition(SwerveModulePosition position) {
        driveMotor.setAngle(position.distanceMeters / DriveConstants.wheelRadius.in(Meters));
        steerMotor.setAngle(position.angle.getRadians());
    }

    @Override
    public Angle getSteerAbsAngle() {
        return steerMotor.getAngularPosition();
    }

    @Override
    public Angle getSteerAngle() {
        return steerMotor.getAngularPosition();
    }

    public AngularVelocity getSteerVelocity() {
        return steerMotor.getAngularVelocity();
    }

    public AngularAcceleration getSteerAcceleration() {
        return steerMotor.getAngularAcceleration();
    }

    public Distance getDrivePosition() {
        return Meters.of(driveMotor.getAngularPositionRad() * DriveConstants.wheelRadius.in(Meters));
    }

    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(driveMotor.getAngularVelocityRadPerSec() * DriveConstants.wheelRadius.in(Meters));
    }

    public LinearAcceleration getDriveAcceleration() {
        return MetersPerSecondPerSecond.of(driveMotor.getAngularAccelerationRadPerSecSq() * DriveConstants.wheelRadius.in(Meters));
    }

    public Voltage getDriveVoltage() {
        return Volts.of(driveMotor.getInputVoltage());
    }

    public Voltage getSteerVoltage() {
        return Volts.of(steerMotor.getInputVoltage());
    }

    public Current getDriveCurrent() {
        return Amps.of(driveMotor.getCurrentDrawAmps());
    }

    public Current getSteerCurrent() {
        return Amps.of(steerMotor.getCurrentDrawAmps());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerAngle()));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
    }
}
