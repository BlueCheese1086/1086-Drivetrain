package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Celsius;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface Module {
    public Angle getSteerAbsAngle();
    public Angle getSteerAngle();
    public AngularVelocity getSteerVelocity();
    public AngularAcceleration getSteerAcceleration();

    public Distance getDrivePosition();
    public LinearVelocity getDriveVelocity();
    public LinearAcceleration getDriveAcceleration();
    
    public Voltage getDriveVoltage();
    public Voltage getSteerVoltage();
    
    public Current getDriveCurrent();
    public Current getSteerCurrent();
    
    public Temperature getDriveTemperature();
    public Temperature getSteerTemperature();
    
    public SwerveModulePosition getModulePosition();
    public SwerveModuleState getModuleState();
    
    public void setState(SwerveModuleState state);

    public void resetPosition();
    public void resetPosition(SwerveModulePosition position);
}