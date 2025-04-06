package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.AdjustableValues;

public class ModuleIOSim implements ModuleIO {
    private int moduleId;

    private DCMotorSim driveMotor;
    private DCMotorSim steerMotor;

    private PIDController driveController;
    private PIDController steerController;

    private SimpleMotorFeedforward driveFFController;
    private SimpleMotorFeedforward steerFFController;

    private SwerveModuleState setpoint = new SwerveModuleState();

    /**
     * Creates a simulated ModuleIO.
     * 
     * @param moduleId The module id used for logging and getting configs.
     */
    public ModuleIOSim(int moduleId) {
        this.moduleId = moduleId;

        driveFFController = new SimpleMotorFeedforward(AdjustableValues.getNumber("Drive_kS_" + moduleId), AdjustableValues.getNumber("Drive_kV_" + moduleId), 0);
        steerFFController = new SimpleMotorFeedforward(AdjustableValues.getNumber("Steer_kS_" + moduleId), AdjustableValues.getNumber("Steer_kV_" + moduleId), 0);

        driveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), DriveConstants.driveMOI, DriveConstants.driveGearRatio), DCMotor.getKrakenX60(1));
        steerMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DriveConstants.krakenX44, DriveConstants.steerMOI, DriveConstants.steerGearRatio), DriveConstants.krakenX44);

        driveController = new PIDController(AdjustableValues.getNumber("Drive_kP_" + moduleId), AdjustableValues.getNumber("Drive_kI_" + moduleId), AdjustableValues.getNumber("Drive_kD_" + moduleId));
        steerController = new PIDController(AdjustableValues.getNumber("Steer_kP_" + moduleId), AdjustableValues.getNumber("Steer_kI_" + moduleId), AdjustableValues.getNumber("Steer_kD_" + moduleId));

        steerController.enableContinuousInput(Math.PI, -Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if (AdjustableValues.hasChanged("Drive_kP_" + moduleId)) driveController.setP(AdjustableValues.getNumber("Drive_kP_" + moduleId));
        if (AdjustableValues.hasChanged("Drive_kI_" + moduleId)) driveController.setI(AdjustableValues.getNumber("Drive_kI_" + moduleId));
        if (AdjustableValues.hasChanged("Drive_kD_" + moduleId)) driveController.setD(AdjustableValues.getNumber("Drive_kD_" + moduleId));
        if (AdjustableValues.hasChanged("Drive_kS_" + moduleId)) driveFFController.setKs(AdjustableValues.getNumber("Drive_kS_" + moduleId));
        if (AdjustableValues.hasChanged("Drive_kV_" + moduleId)) driveFFController.setKv(AdjustableValues.getNumber("Drive_kV_" + moduleId));

        if (AdjustableValues.hasChanged("Steer_kP_" + moduleId)) steerController.setP(AdjustableValues.getNumber("Steer_kP_" + moduleId));
        if (AdjustableValues.hasChanged("Steer_kI_" + moduleId)) steerController.setI(AdjustableValues.getNumber("Steer_kI_" + moduleId));
        if (AdjustableValues.hasChanged("Steer_kD_" + moduleId)) steerController.setD(AdjustableValues.getNumber("Steer_kD_" + moduleId));
        if (AdjustableValues.hasChanged("Steer_kS_" + moduleId)) steerFFController.setKs(AdjustableValues.getNumber("Steer_kS_" + moduleId));
        if (AdjustableValues.hasChanged("Steer_kV_" + moduleId)) steerFFController.setKv(AdjustableValues.getNumber("Steer_kV_" + moduleId));

        double driveVolts = MathUtil.clamp(driveController.calculate(inputs.driveVelocity.in(MetersPerSecond)) + driveMotor.getInputVoltage() + driveFFController.calculate(setpoint.speedMetersPerSecond), -12, 12);
        double steerVolts = MathUtil.clamp(steerController.calculate(inputs.steerAngle.getRadians()) + steerFFController.calculate((setpoint.angle.getRadians() - steerMotor.getAngularPosition().in(Radians)) / 0.02), -12, 12);

        driveMotor.setInputVoltage(driveVolts);
        steerMotor.setInputVoltage(steerVolts);

        driveMotor.update(0.02);
        steerMotor.update(0.02);

        inputs.steerAbsAngle = new Rotation2d(steerMotor.getAngularPosition());

        inputs.steerAngle = new Rotation2d(steerMotor.getAngularPosition());
        inputs.steerVelocity = steerMotor.getAngularVelocity();
        inputs.steerAcceleration = steerMotor.getAngularAcceleration();

        inputs.driveDistance = Meters.of(driveMotor.getAngularPositionRad() * DriveConstants.wheelRadius.in(Meters));
        inputs.driveVelocity = MetersPerSecond.of(driveMotor.getAngularVelocityRadPerSec() * DriveConstants.wheelRadius.in(Meters));;
        inputs.driveAcceleration = MetersPerSecondPerSecond.of(driveMotor.getAngularAccelerationRadPerSecSq() * DriveConstants.wheelRadius.in(Meters));;

        inputs.driveVoltage = Volts.of(driveMotor.getInputVoltage());
        inputs.steerVoltage = Volts.of(steerMotor.getInputVoltage());

        inputs.driveCurrent = Amps.of(driveMotor.getCurrentDrawAmps());
        inputs.steerCurrent = Amps.of(steerMotor.getCurrentDrawAmps());

        inputs.modulePosition = new SwerveModulePosition(inputs.driveDistance, inputs.steerAngle);
        inputs.moduleState = new SwerveModuleState(inputs.driveVelocity, inputs.steerAngle);
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
}