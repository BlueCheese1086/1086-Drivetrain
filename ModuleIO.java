package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public class ModuleIOInputs {
        Rotation2d steerAbsAngle = new Rotation2d();

        Rotation2d steerAngle = new Rotation2d();
        double steerVelocity = 0; // Radians / Second
        double steerAcceleration = 0; // Radians / Second^2

        double driveDistance = 0; // Meters
        double driveVelocity = 0; // Meters / Second
        double driveAcceleration = 0; // Meters / Second^2

        double driveVoltage = 0; // Volts
        double steerVoltage = 0; // Volts

        double driveCurrent = 0; // Amps
        double steerCurrent = 0; // Amps

        double driveTemperature = 0; // Celsius
        double steerTemperature = 0; // Celsius

        SwerveModulePosition modulePosition = new SwerveModulePosition();
        SwerveModuleState moduleState = new SwerveModuleState();
    }

    public void updateInputs(ModuleIOInputs inputs);

    public void setState(SwerveModuleState state);

    public void resetPosition(SwerveModulePosition position);
}