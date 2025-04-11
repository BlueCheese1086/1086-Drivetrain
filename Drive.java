package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.util.VisionResult;
import frc.robot.util.AdjustableValues;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private ModuleIO[] modules;
    private ModuleIOInputsAutoLogged[] moduleInputs;
    private SwerveModuleState[] states;
    private SwerveModulePosition[] positions;

    public PIDController xController = new PIDController(
        AdjustableValues.getNumber("X_kP"),
        AdjustableValues.getNumber("X_kI"),
        AdjustableValues.getNumber("X_kD"));

    public PIDController yController = new PIDController(
        AdjustableValues.getNumber("Y_kP"),
        AdjustableValues.getNumber("Y_kI"),
        AdjustableValues.getNumber("Y_kD"));

    public PIDController thetaController = new PIDController(
        AdjustableValues.getNumber("Theta_kP"),
        AdjustableValues.getNumber("Theta_kI"),
        AdjustableValues.getNumber("Theta_kD"));

    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;

    // Subsystem depencies
    private Gyro gyro;
    private Vision vision;

    // Configs for closed loop control
    private boolean headingLocked;
    private Rotation2d lockedAngle;

    private Rotation2d heading = new Rotation2d();

    /**
     * Creates a new Drivetrain subsystem.
     * 
     * @param gyro The gyro instance to get heading from.
     * @param vision The vision instance to get pose estimates from.
     * @param modules The module IOs to drive on.
    */
    public Drive(Gyro gyro, Vision vision, ModuleIO... modules) {
        // SwerveDrivetrainConstants dConst = new SwerveDrivetrainConstants();
    
        // dConst.Pigeon2Configs = new Pigeon2Configuration();
        // dConst.Pigeon2Id = RobotMap.GYRO_Pigeon2Id; // Same as default...  Should I leave it?
        
        // SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration> mConst = new SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration>();

        // int moduleId = 0;
        
        // Slot0Configs driveSlot0 = new Slot0Configs();
        // driveSlot0.kP = DriveConstants.kPDriveDefault;
        // driveSlot0.kI = DriveConstants.kIDriveDefault;
        // driveSlot0.kD = DriveConstants.kDDriveDefault;
        // driveSlot0.kS = DriveConstants.kSDriveDefault;
        // driveSlot0.kV = DriveConstants.kVDriveDefault;

        // Slot0Configs steerSlot0 = new Slot0Configs();
        // steerSlot0.kP = DriveConstants.kPSteerDefault;
        // steerSlot0.kI = DriveConstants.kISteerDefault;
        // steerSlot0.kD = DriveConstants.kDSteerDefault;
        // steerSlot0.kS = DriveConstants.kSSteerDefault;
        // steerSlot0.kV = DriveConstants.kVSteerDefault;

        // TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        // driveConfig.Audio.AllowMusicDurDisable = false;
        // driveConfig.Audio.BeepOnBoot = true;
        // driveConfig.Audio.BeepOnConfig = true;
        // driveConfig.ClosedLoopGeneral.ContinuousWrap = true;
        // driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        // driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
        // driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.driveCurrentLimit.in(Amps);
        // driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // driveConfig.DifferentialConstants.PeakDifferentialDutyCycle = 1;
        // driveConfig.DifferentialConstants.PeakDifferentialVoltage = 12;
        // driveConfig.DifferentialSensors.DifferentialRemoteSensorID = (int) DriveConstants.moduleConfigs[moduleId][2];
        // driveConfig.DifferentialSensors.DifferentialSensorSource = DifferentialSensorSourceValue.RemoteCANcoder;
        // driveConfig.Feedback.FeedbackRemoteSensorID = (int) DriveConstants.moduleConfigs[moduleId][2];
        // driveConfig.Feedback.FeedbackRotorOffset; // Not sure if this needs to be set... The offset will be set in the CANcoder config
        // driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // driveConfig.MotorOutput.NeutralMode;
        // driveConfig.MotorOutput.PeakForwardDutyCycle;
        // driveConfig.MotorOutput.PeakReverseDutyCycle;
        // driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod;
        // driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod;
        // driveConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable;
        // driveConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold;
        // driveConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable;
        // driveConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
        // driveConfig.Voltage.PeakForwardVoltage;
        // driveConfig.Voltage.PeakReverseVoltage;
        // driveConfig.Voltage.SupplyVoltageTimeConstant;

        // TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        // CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        // mConst.CouplingGearRatio = 0; //?
        // mConst.DriveFrictionVoltage = 0.25; //?
        // mConst.DriveInertia = DriveConstants.driveMOI;
        // mConst.DriveMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // mConst.DriveMotorGains = driveSlot0;
        // mConst.DriveMotorGearRatio = DriveConstants.driveGearRatio;
        // mConst.DriveMotorId = (int) DriveConstants.moduleConfigs[moduleId][0];
        // mConst.DriveMotorInitialConfigs = new TalonFXConfiguration(); //?
        // mConst.DriveMotorInverted = true;
        // mConst.DriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // mConst.EncoderId = (int) DriveConstants.moduleConfigs[moduleId][2];
        // mConst.EncoderInitialConfigs = new CANcoderConfiguration(); //?
        // mConst.EncoderInverted = false;
        // mConst.EncoderOffset = DriveConstants.moduleConfigs[moduleId][3];
        // mConst.FeedbackSource = SteerFeedbackType.RemoteCANcoder;
        // mConst.LocationX = DriveConstants.flModuleOffset.getX();
        // mConst.LocationY = DriveConstants.flModuleOffset.getY();
        // mConst.SlipCurrent = 120; //?
        // mConst.SpeedAt12Volts = DriveConstants.maxLinearVelocity.in(MetersPerSecond);
        // mConst.SteerFrictionVoltage = 0.25; //?
        // mConst.SteerInertia = DriveConstants.steerMOI;
        // mConst.SteerMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // mConst.SteerMotorGains = steerSlot0;
        // mConst.SteerMotorGearRatio = DriveConstants.steerGearRatio;
        // mConst.SteerMotorId = (int) DriveConstants.moduleConfigs[moduleId][1];
        // mConst.SteerMotorInitialConfigs = new TalonFXConfiguration(); //?
        // mConst.SteerMotorInverted = false;
        // mConst.SteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
        // mConst.WheelRadius = DriveConstants.wheelRadius.in(Meters);



        // SwerveDrivetrain<TalonFX,TalonFX,CANcoder> dt = new SwerveDrivetrain<TalonFX,TalonFX,CANcoder>(TalonFX::new, TalonFX::new, CANcoder::new, dConst, mConst);
        // SwerveModule<TalonFX,TalonFX,CANcoder> module = new SwerveModule<TalonFX,TalonFX,CANcoder>(TalonFX::new, TalonFX::new, CANcoder::new, consts, "rio", 1, 0);


        // END OF TESTING STUFF, REAL CODE NOW
        System.out.println("Drivetrain initialized");

        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        thetaController.setTolerance(0.01);

        // Saving subsystems
        this.gyro = gyro;
        this.vision = vision;

        this.modules = modules;
        this.moduleInputs = new ModuleIOInputsAutoLogged[modules.length];
        this.states = new SwerveModuleState[modules.length];
        this.positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < modules.length; i++) {
            moduleInputs[i] = new ModuleIOInputsAutoLogged();
            states[i] = moduleInputs[i].moduleState;
            positions[i] = moduleInputs[i].modulePosition;
        }

        /*
         *  BL | FL 
         *     |    
         * ---------
         *     |    
         *  BR | FR 
         */

        kinematics = new SwerveDriveKinematics(DriveConstants.translations);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, heading, positions, new Pose2d());

        // Configuring SysID
        new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::driveVolts, this::sysIdLog, this, "SwerveDrive"));

        // Configuring Pathplanner
        AutoBuilder.configure(this::getPose, this::resetPose, this::getSpeeds, this::drive,
            new PPHolonomicDriveController(
                new PIDConstants(DriveConstants.kPDriveDefault, DriveConstants.kIDriveDefault, DriveConstants.kDDriveDefault),
                new PIDConstants(DriveConstants.kPSteerDefault, DriveConstants.kISteerDefault, DriveConstants.kDSteerDefault)
            ),
            new RobotConfig(
                DriveConstants.robotMass, DriveConstants.robotMOI,
                new ModuleConfig(DriveConstants.wheelRadius, DriveConstants.maxLinearVelocity, 1, DCMotor.getKrakenX60(1).withReduction(DriveConstants.driveGearRatio), DriveConstants.driveCurrentLimit, 2), 
                DriveConstants.translations),
            () -> (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)),
            this);

        // Configuring Choreo
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * DO NOT USE FOR ANYTHING OTHER THAN SYSID!!!
     * THIS FUNCTION DOES NOT CONTROL THE TURN MOTOR
     * 
     * @param volts
     */
    public void driveVolts(Voltage volts) {

    }

    /**
     * This function logs values from sysId.
     * It only logs values from the drive motors.
     * 
     * @param log The log structure to apply changes to.
     */
    public void sysIdLog(SysIdRoutineLog log) {
        for (int i = 0; i < modules.length; i++) {
            log.motor("Module" + i + "_Drive")
                .current(moduleInputs[i].driveCurrent)
                .voltage(moduleInputs[i].driveVoltage)
                .linearAcceleration(moduleInputs[i].driveAcceleration)
                .linearVelocity(moduleInputs[i].driveVelocity)
                .linearPosition(moduleInputs[i].driveDistance);
        }
    }

    /**
     * Runs once every tick the subsystem is active.
     * 
     * It updates the module IO inputs, the estimated pose, and the module states/positions.
     * It also logs multiple values specific to the drivetrain subsystem.
     */
    @Override
    public void periodic() {
        if (AdjustableValues.hasChanged("X_kP")) xController.setP(AdjustableValues.getNumber("X_kP"));
        if (AdjustableValues.hasChanged("X_kI")) xController.setI(AdjustableValues.getNumber("X_kI"));
        if (AdjustableValues.hasChanged("X_kD")) xController.setD(AdjustableValues.getNumber("X_kD"));

        if (AdjustableValues.hasChanged("Y_kP")) yController.setP(AdjustableValues.getNumber("Y_kP"));
        if (AdjustableValues.hasChanged("Y_kI")) yController.setI(AdjustableValues.getNumber("Y_kI"));
        if (AdjustableValues.hasChanged("Y_kD")) yController.setD(AdjustableValues.getNumber("Y_kD"));

        if (AdjustableValues.hasChanged("Theta_kP")) thetaController.setP(AdjustableValues.getNumber("Theta_kP"));
        if (AdjustableValues.hasChanged("Theta_kI")) thetaController.setI(AdjustableValues.getNumber("Theta_kI"));
        if (AdjustableValues.hasChanged("Theta_kD")) thetaController.setD(AdjustableValues.getNumber("Theta_kD"));

        SwerveModulePosition[] oldPositions = positions.clone();

        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs(moduleInputs[i]);

            Logger.processInputs("/RealOutputs/Subsystems/Drivetrain/Module" + i, moduleInputs[i]);
            
            states[i] = moduleInputs[i].moduleState;
            positions[i] = moduleInputs[i].modulePosition;
        }


        if (gyro.isConnected()) {
            heading = gyro.getHeading();
        } else {
            SwerveModulePosition[] deltas = new SwerveModulePosition[4];

            for (int i = 0; i < 4; i++) {
                deltas[i] = new SwerveModulePosition(positions[i].distanceMeters - oldPositions[i].distanceMeters, positions[i].angle);
            }

            Twist2d twist = kinematics.toTwist2d(deltas);

            heading = heading.plus(new Rotation2d(twist.dtheta));
        }

        poseEstimator.update(heading, positions);

        for (VisionResult result : vision.getUnreadResults()) {
            poseEstimator.addVisionMeasurement(result.getPose2d(), result.getTimestamp());
        }

        Logger.recordOutput("/Subsystems/Drivetrain/HeadingLocked", headingLocked);
        Logger.recordOutput("/Subsystems/Drivetrain/HeadingSetpoint", lockedAngle);

        Logger.recordOutput("/Subsystems/Drivetrain/States/Actual", states);
        Logger.recordOutput("/Subsystems/Drivetrain/Positions/Actual", positions);

        Logger.recordOutput("/Subsystems/Drivetrain/RobotPose", poseEstimator.getEstimatedPosition());
    }

    /** Gets the current pose. */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the pose
     * 
     * @param newPose The new pose to go to.
     */
    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPosition(heading, positions, newPose);
    }

    /** Gets the current heading. */
    public Rotation2d getHeading() {
        return heading;
    }

    /** Gets the current wheel speeds. */
    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(states);
    }

    /**
     * Drives the robot according to some ChassisSpeeds.
     * 
     * If the heading is locked, omega is ignored.
     */
    public void drive(ChassisSpeeds speeds) {
        if (headingLocked) {
            Rotation2d angle = (lockedAngle == null) ? heading : lockedAngle;
            speeds.omegaRadiansPerSecond = thetaController.calculate(heading.getRadians(), angle.getRadians());
        }

        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxLinearVelocity);

        for (int i = 0; i < modules.length; i++) {
            desiredStates[i].optimize(moduleInputs[i].steerAngle);
            desiredStates[i].cosineScale(moduleInputs[i].steerAngle);

            modules[i].setState(desiredStates[i]);
        }

        Logger.recordOutput("/Subsystems/Drivetrain/States/Setpoint", desiredStates);
        Logger.recordOutput("/Subsystems/Drivetrain/Speeds/Setpoint", speeds);
    }

    /** Locks the heading */
    public void setHeadingLock(boolean lock) {
        headingLocked = lock;
    }

    /**
     * Locks the heading.
     * 
     * @param angle The angle to lock the heading to.
     */
    public void setHeadingLock(boolean lock, Rotation2d angle) {
        headingLocked = lock;

        setLockedAngle(angle);
    }

    public boolean isLocked() {
        return headingLocked;
    }

    /**
     * Gets the angle that the robot is locked onto.
     * 
     * Returns null if the heading is not locked.
     */
    public Rotation2d getLockedAngle() {
        return lockedAngle;
    }

    /** Sets the angle that the robot will lock to. */
    public void setLockedAngle(Rotation2d angle) {
        lockedAngle = angle;
    }

    /** Gets the kinematics of the robot. */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
    
    /** Follows a choreo trajectory. */
    public void followTrajectory(SwerveSample sample) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(sample.getChassisSpeeds(), heading));
    }

    /** Sets the states of each module to an "X" pattern. */
    public void xStates() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(DriveConstants.xStates[i]);
        }
    }
}