
package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.AdjustableValues;
import org.littletonrobotics.junction.Logger;

public class ModuleSim extends Module {
    private int moduleId;

    private DCMotorSim driveMotor;
    private DCMotorSim steerMotor;

    private PIDController driveController;
    private PIDController steerController;

    private SimpleMotorFeedforward driveFFController;
    private SimpleMotorFeedforward steerFFController;

    private StructTopic<SwerveModulePosition> modulePositionTopic;
    private StructTopic<SwerveModuleState> moduleStateTopic;

    /**
     * Creates a simulated ModuleIO.
     *
     * @param moduleId The module id used for logging and getting configs.
     */
    public ModuleSim(int moduleId) {
        this.moduleId = moduleId;

        driveFFController = new SimpleMotorFeedforward(0, 0, 0);//AdjustableValues.getNumber("Drive_kS_" + moduleId), AdjustableValues.getNumber("Drive_kV_" + moduleId), 0);
        steerFFController = new SimpleMotorFeedforward(0, 0, 0);//AdjustableValues.getNumber("Steer_kS_" + moduleId), AdjustableValues.getNumber("Steer_kV_" + moduleId), 0);

        driveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), DriveConstants.driveMOI, DriveConstants.driveGearRatio), DCMotor.getKrakenX60(1));
        steerMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DriveConstants.krakenX44, DriveConstants.steerMOI, DriveConstants.steerGearRatio), DriveConstants.krakenX44);

        driveController = new PIDController(0, 0, 0);//AdjustableValues.getNumber("Drive_kP_" + moduleId), AdjustableValues.getNumber("Drive_kI_" + moduleId), AdjustableValues.getNumber("Drive_kD_" + moduleId));
        steerController = new PIDController(0, 0, 0);//AdjustableValues.getNumber("Steer_kP_" + moduleId), AdjustableValues.getNumber("Steer_kI_" + moduleId), AdjustableValues.getNumber("Steer_kD_" + moduleId));

        steerController.enableContinuousInput(Math.PI, -Math.PI);

        // modulePositionTopic = NetworkTableInstance.getDefault().getStructTopic("/Drive/Module" + moduleId + "/Position", SwerveModulePosition.struct);
        // moduleStateTopic = NetworkTableInstance.getDefault().getStructTopic("/Drive/Module" + moduleId + "/State", SwerveModuleState.struct);
    }

    @Override
    public void periodic() {
        // if (AdjustableValues.hasChanged("Drive_kP_" + moduleId)) driveController.setP(AdjustableValues.getNumber("Drive_kP_" + moduleId));
        // if (AdjustableValues.hasChanged("Drive_kI_" + moduleId)) driveController.setI(AdjustableValues.getNumber("Drive_kI_" + moduleId));
        // if (AdjustableValues.hasChanged("Drive_kD_" + moduleId)) driveController.setD(AdjustableValues.getNumber("Drive_kD_" + moduleId));
        // if (AdjustableValues.hasChanged("Drive_kS_" + moduleId)) driveFFController.setKs(AdjustableValues.getNumber("Drive_kS_" + moduleId));
        // if (AdjustableValues.hasChanged("Drive_kV_" + moduleId)) driveFFController.setKv(AdjustableValues.getNumber("Drive_kV_" + moduleId));

        // if (AdjustableValues.hasChanged("Steer_kP_" + moduleId)) steerController.setP(AdjustableValues.getNumber("Steer_kP_" + moduleId));
        // if (AdjustableValues.hasChanged("Steer_kI_" + moduleId)) steerController.setI(AdjustableValues.getNumber("Steer_kI_" + moduleId));
        // if (AdjustableValues.hasChanged("Steer_kD_" + moduleId)) steerController.setD(AdjustableValues.getNumber("Steer_kD_" + moduleId));
        // if (AdjustableValues.hasChanged("Steer_kS_" + moduleId)) steerFFController.setKs(AdjustableValues.getNumber("Steer_kS_" + moduleId));
        // if (AdjustableValues.hasChanged("Steer_kV_" + moduleId)) steerFFController.setKv(AdjustableValues.getNumber("Steer_kV_" + moduleId));

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

        SmartDashboard.putNumber("/Drive/Module" + moduleId + "/SteerAbsPosition", getSteerAbsAngle().in(Degrees));
        SmartDashboard.putNumber("/Drive/Module" + moduleId + "/SteerPosition/Actual", getSteerAngle().in(Degrees));
        SmartDashboard.putNumber("/Drive/Module" + moduleId + "/SteerVelocity", getSteerVelocity().in(DegreesPerSecond));
        SmartDashboard.putNumber("/Drive/Module" + moduleId + "/SteerAcceleration", getSteerAcceleration().in(DegreesPerSecondPerSecond));

        SmartDashboard.putNumber("/Drive/Module" + moduleId + "/DrivePosition", getDrivePosition().in(Meters));
        SmartDashboard.putNumber("/Drive/Module" + moduleId + "/DriveVelocity/Actual", getDriveVelocity().in(MetersPerSecond));
        SmartDashboard.putNumber("/Drive/Module" + moduleId + "/DriveAcceleration", getDriveAcceleration().in(MetersPerSecondPerSecond));

        SmartDashboard.putNumber("/Drive/Module" + moduleId + "/DriveVoltage", getSteerVoltage().in(Volts));
        SmartDashboard.putNumber("/Drive/Module" + moduleId + "/SteerVoltage", getDriveVoltage().in(Volts));

        SmartDashboard.putNumber("/Drive/Module" + moduleId + "/DriveCurrent", getDriveCurrent().in(Amps));
        SmartDashboard.putNumber("/Drive/Module" + moduleId + "/SteerCurrent", getSteerCurrent().in(Amps));

        modulePositionTopic.publish().set(getModulePosition());
        moduleStateTopic.publish().set(getModuleState());
    }

    @Override
    public void setState(SwerveModuleState state) {
        driveController.setSetpoint(state.speedMetersPerSecond / DriveConstants.wheelRadius.in(Meters));
        steerController.setSetpoint(state.angle.getRadians());
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
