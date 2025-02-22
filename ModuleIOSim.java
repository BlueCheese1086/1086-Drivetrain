package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class ModuleIOSim implements ModuleIO {
    private int moduleId;

    private DCMotorSim driveMotor;
    private DCMotorSim steerMotor;

    private PIDController driveController;
    private PIDController steerController;

    private ModuleIOInputsAutoLogged inputs;

    public ModuleIOSim(int moduleId) {
        this.moduleId = moduleId;

        driveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), DriveConstants.driveMOI, DriveConstants.driveGearRatio), DCMotor.getKrakenX60(1));
        steerMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), DriveConstants.steerMOI, DriveConstants.steerGearRatio), DCMotor.getKrakenX60(1));

        driveController = new PIDController(DriveConstants.kPDriveSim, DriveConstants.kIDriveSim, DriveConstants.kDDriveSim);
        steerController = new PIDController(DriveConstants.kPSteerSim, DriveConstants.kISteerSim, DriveConstants.kDSteerSim);
        
        steerController.enableContinuousInput(0, Math.PI * 2);

        inputs = new ModuleIOInputsAutoLogged();
    }

    @Override
    public void updateInputs() {
        double driveVolts = MathUtil.clamp(driveController.calculate(driveMotor.getAngularVelocityRadPerSec() * DriveConstants.wheelRadius.in(Meters)) + driveMotor.getInputVoltage(), -12, 12);
        double steerVolts = MathUtil.clamp(steerController.calculate(getAngle().getRadians()), -12, 12);

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

        Logger.processInputs(String.format("Module%d_Sim", moduleId), inputs);
    }

    @Override
    public void setState(SwerveModuleState state) {
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
        return new Rotation2d(steerMotor.getAngularPosition());
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
        return Meters.of(driveMotor.getAngularPositionRad() * DriveConstants.wheelRadius.in(Meters));
    }

    @Override
    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(driveMotor.getAngularVelocityRadPerSec() * DriveConstants.wheelRadius.in(Meters));
    }

    @Override
    public LinearAcceleration getDriveAcceleration() {
        return MetersPerSecondPerSecond.of(driveMotor.getAngularAccelerationRadPerSecSq() * DriveConstants.wheelRadius.in(Meters));
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