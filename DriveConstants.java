package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static class Characterization {
        public static final double ffDelay = 2.0; // Secs
        public static final double ffRampRate = 0.1; // Volts/Sec
        public static final double wheelRadiusMaxVelocity = 0.25; // Rad/Sec
        public static final double wheelRadiusRampRate = 0.05; // Rad/Sec^2
    }

    public static class ControlConstants {
        public static final double kPAngle = 5.0;
        public static final double kDAngle = 0.4;
        public static final double angleMaxVelocity = 5.0;
        public static final double angleMaxAccel = 20.0;
        public static final double maxAccel = 4.75;
        public static final double maxVelocity = 3.25;
    }

    public static final double driveGearing = 6.746;
    public static final double turnGearing = 150.0 / 7.0;

    public static final double drivePositionConversionFactor = 2.0 * Math.PI * Units.inchesToMeters(2.00)
            / driveGearing;
    public static final double turnPositionConversionFactor = 2.0 * Math.PI / turnGearing;

    public static final double driveVelocityFactor = drivePositionConversionFactor / 60.0;
    public static final double turnVelocityFactor = turnPositionConversionFactor / 60.0;
    public static final boolean tuningMode = false;
};
