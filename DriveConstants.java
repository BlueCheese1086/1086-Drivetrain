package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {
    public static class Characterization {
        public static final double ffDelay = 2.0; // Secs
        public static final double ffRampRate = 0.1; // Volts/Sec
        public static final double wheelRadiusMaxVelocity = 0.25; // Rad/Sec
        public static final double wheelRadiusRampRate = 0.05; // Rad/Sec^2
    }

    public static final double kPX = 10;
    public static final double kIX = 0;
    public static final double kDX = 0;

    public static final double kPY = 10;
    public static final double kIY = 0;
    public static final double kDY = 0;

    public static final double kPTheta = 5;
    public static final double kITheta = 0;
    public static final double kDTheta = 0.4;

    public static final double driveGearing = 6.746;
    public static final double turnGearing = 150.0 / 7.0;

    public static final Distance transX = Inches.of(11.375);
    public static final Distance transY = Inches.of(11.375);
    public static final Distance wheelRadius = Inches.of(2);
    public static final Distance robotRadius = Meters.of(Math.hypot(transX.in(Meters), transY.in(Meters)));

    public static final double drivePositionConversionFactor = 2.0 * Math.PI * wheelRadius.in(Meters) / driveGearing;
    public static final double driveVelocityFactor = drivePositionConversionFactor / 60.0;

    public static final double turnPositionConversionFactor = 2.0 * Math.PI / turnGearing;
    public static final double turnVelocityFactor = turnPositionConversionFactor / 60.0;

    public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(4.73);
    public static final LinearAcceleration maxLinearAcceleration = MetersPerSecondPerSecond.of(4.75);

    public static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(5);
    public static final AngularAcceleration maxAngularAcceleration = RadiansPerSecondPerSecond.of(20);

    public static final PathConstraints ppConstraints = new PathConstraints(
            DriveConstants.maxLinearVelocity,
            DriveConstants.maxLinearAcceleration,
            DriveConstants.maxAngularVelocity,
            DriveConstants.maxAngularAcceleration);
}