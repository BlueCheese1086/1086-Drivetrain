package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public class ModuleIOInputs {
        SwerveModulePosition modulePosition = new SwerveModulePosition();
        SwerveModuleState moduleState = new SwerveModuleState();

        Rotation2d steerAbsAngle = new Rotation2d();

        Rotation2d steerAngle = new Rotation2d();
        AngularVelocity steerVelocity = RotationsPerSecond.zero();
        AngularAcceleration steerAcceleration = RotationsPerSecondPerSecond.zero();

        Distance driveDistance = Meters.zero();
        LinearVelocity driveVelocity = MetersPerSecond.zero();
        LinearAcceleration driveAcceleration = MetersPerSecondPerSecond.zero();

        Voltage driveVoltage = Volts.zero();
        Voltage steerVoltage = Volts.zero();
        
        Current driveCurrent = Amps.zero();
        Current steerCurrent = Amps.zero();

        Temperature driveTemperature = Celsius.zero();
        Temperature steerTemperature = Celsius.zero();
    }

    public void updateInputs();

    public void setState(SwerveModuleState state);
    public void resetPosition(SwerveModulePosition position);

    public SwerveModuleState getState();
    public SwerveModulePosition getPosition();

    public Rotation2d getAbsoluteAngle();

    public Rotation2d getAngle();
    public AngularVelocity getSteerVelocity();
    public AngularAcceleration getSteerAcceleration();

    public Distance getDistance();
    public LinearVelocity getDriveVelocity();
    public LinearAcceleration getDriveAcceleration();
    
    public Voltage getDriveVoltage();
    public Voltage getSteerVoltage();
    
    public Current getDriveCurrent();
    public Current getSteerCurrent();

    public Temperature getDriveTemperature();
    public Temperature getSteerTemperature();
}