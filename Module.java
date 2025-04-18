package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase {
    public void setState(SwerveModuleState state) {}

    public void resetPosition(SwerveModulePosition position) {}

    public Angle getSteerAbsAngle() {
        return Radians.zero();
    }

    public Angle getSteerAngle() {
        return Radians.zero();
    }

    public AngularVelocity getSteerVelocity() {
        return RadiansPerSecond.zero();
    }

    public AngularAcceleration getSteerAcceleration() {
        return RadiansPerSecondPerSecond.zero();
    }

    public Distance getDrivePosition() {
        return Meters.zero();
    }

    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.zero();
    }

    public LinearAcceleration getDriveAcceleration() {
        return MetersPerSecondPerSecond.zero();
    }

    public Voltage getDriveVoltage() {
        return Volts.zero();
    }

    public Voltage getSteerVoltage() {
        return Volts.zero();
    }

    public Current getDriveCurrent() {
        return Amps.zero();
    }

    public Current getSteerCurrent() {
        return Amps.zero();
    }

    public Temperature getDriveTemperature() {
        return Celsius.zero();
    }

    public Temperature getSteerTemperature() {
        return Celsius.zero();
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition();
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState();
    }

}