package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.AdjustableValues;
import org.littletonrobotics.junction.Logger;

public class ModuleIOSim implements ModuleIO {
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

        Logger.recordOutput("/Drive/PIDS/Drive_kP", driveController.getP());
        Logger.recordOutput("/Drive/PIDS/Drive_kI", driveController.getI());
        Logger.recordOutput("/Drive/PIDS/Drive_kD", driveController.getD());
        Logger.recordOutput("/Drive/PIDS/Drive_kS", driveFFController.getKs());
        Logger.recordOutput("/Drive/PIDS/Drive_kV", driveFFController.getKv());

        Logger.recordOutput("/Drive/PIDS/Steer_kP", steerController.getP());
        Logger.recordOutput("/Drive/PIDS/Steer_kI", steerController.getI());
        Logger.recordOutput("/Drive/PIDS/Steer_kD", steerController.getD());
        Logger.recordOutput("/Drive/PIDS/Steer_kS", steerFFController.getKs());
        Logger.recordOutput("/Drive/PIDS/Steer_kV", steerFFController.getKv());

        double driveVolts = driveController.calculate(driveMotor.getAngularVelocityRadPerSec()) + driveFFController.calculate(driveController.getSetpoint());
        double steerVolts = steerController.calculate(steerMotor.getAngularPositionRad()) + steerFFController.calculate((steerController.getSetpoint() - steerMotor.getAngularPositionRad()) / 0.02);

        driveMotor.setInputVoltage(driveVolts);
        steerMotor.setInputVoltage(steerVolts);

        driveMotor.update(0.02);
        steerMotor.update(0.02);

        inputs.steerAbsAngle = new Rotation2d(steerMotor.getAngularPosition());

        inputs.steerAngle = new Rotation2d(steerMotor.getAngularPosition());
        inputs.steerVelocity = steerMotor.getAngularVelocityRadPerSec();
        inputs.steerAcceleration = steerMotor.getAngularAccelerationRadPerSecSq();

        inputs.driveDistance = driveMotor.getAngularPositionRad() * DriveConstants.wheelRadius;
        inputs.driveVelocity = driveMotor.getAngularVelocityRadPerSec() * DriveConstants.wheelRadius;
        inputs.driveAcceleration = driveMotor.getAngularAccelerationRadPerSecSq() * DriveConstants.wheelRadius;

        inputs.driveVoltage = driveVolts;
        inputs.steerVoltage = steerVolts;

        inputs.driveCurrent = driveMotor.getCurrentDrawAmps();
        inputs.steerCurrent = steerMotor.getCurrentDrawAmps();

        inputs.modulePosition = new SwerveModulePosition(inputs.driveDistance, inputs.steerAngle);
        inputs.moduleState = new SwerveModuleState(inputs.driveVelocity, inputs.steerAngle);
    }

    @Override
    public void setState(SwerveModuleState state) {
        driveController.setSetpoint(state.speedMetersPerSecond / DriveConstants.wheelRadius);
        steerController.setSetpoint(state.angle.getRadians());
    }

    @Override
    public void resetPosition(SwerveModulePosition position) {
        driveMotor.setAngle(position.distanceMeters / DriveConstants.wheelRadius);
        steerMotor.setAngle(position.angle.getRadians());
    }
}