# 1086 Drivetrain Subsystem
This is the AdvantageKit-compliant Drivetrain subsystem developed by team Blue Cheese 1086.

It supports Swerve drivetrains that use CANCoders as absolute encoders and the following Motors:
* TalonFX
* SparkMax

To use the subsytem in your code, just clone the repository to a "drivetrain" folder.

To make it work, define all constants below.
Directories are not capitalized, classes are.
Constants used:
frc.robot.Constants.DriveConstants.maxLinearVelocity: The max drive velocity of the robot as a LinearVelocity.
frc.robot.Constants.DriveConstants.maxLinearAcceleration: The max drive acceleration of the robot as a LinearAcceleration.
frc.robot.Constants.DriveConstants.maxAngularVelocity: The max turn velocity of the robot as an AngularVelocity.
frc.robot.Constants.DriveConstants.maxAngularAcceleration: The max turn acceleration of the robot as an AngularAcceleration.
frc.robot.Constants.DriveConstants.driveMOI: The moment of inertia for the drive motor as a double.
frc.robot.Constants.DriveConstants.steerMOI: The moment of inertia for the steer motor as a double.
frc.robot.Constants.DriveConstants.driveGearRatio: The gear ratio for the drive motor as a double.
frc.robot.Constants.DriveConstants.steerGearRatio: The gear ratio for the steer motor as a double.
frc.robot.Constants.DriveConstants.driveCurrentLimit: The current limit for the drive motor as a Current.
frc.robot.Constants.DriveConstants.steerCurrentLimit: The current limit for the steer motor as a Current.
frc.robot.Constants.DriveConstants.metersPerRotation: A conversion factor for rotations to meters as a double.
frc.robot.Constants.DriveConstants.wheelRadius: The wheel radius as a Distance.
frc.robot.Constants.DriveConstants.translations: An array of Translation2ds that are the distance of each module from the center of the robot.
frc.robot.Constants.DriveConstants.robotMass: The mass of the robot as a Mass.
frc.robot.Constants.DriveConstants.moduleConfigs: A 2d array of doubles where each row has the drive motor id, steer motor id, absolute encoder id, and absolute encoder offset.
frc.robot.Constants.DriveConstants.xStates: An array of SwerveModuleStates that are the desired states to have the robot make an x pattern.
frc.robot.Constants.PIDValues.kPDrive: The kP component of the drive PID controller as a double.
frc.robot.Constants.PIDValues.kIDrive: The kI component of the drive PID controller as a double.
frc.robot.Constants.PIDValues.kDDrive: The kD component of the drive PID controller as a double.
frc.robot.Constants.PIDValues.kPSteer: The kP component of the steer PID controller as a double.
frc.robot.Constants.PIDValues.kISteer: The kI component of the steer PID controller as a double.
frc.robot.Constants.PIDValues.kDSteer: The kD component of the steer PID controller as a double.

(Only for 2025!)
frc.robot.Constants.Poses.REEF_Side1Left:  The position that the robot should be at to be on the left side of side 1 on the reef.
frc.robot.Constants.Poses.REEF_Side1Right: The position that the robot should be at to be on the right side of side 1 on the reef.
frc.robot.Constants.Poses.REEF_Side2Left:  The position that the robot should be at to be on the left side of side 2 on the reef.
frc.robot.Constants.Poses.REEF_Side2Right: The position that the robot should be at to be on the right side of side 2 on the reef.
frc.robot.Constants.Poses.REEF_Side3Left:  The position that the robot should be at to be on the left side of side 3 on the reef.
frc.robot.Constants.Poses.REEF_Side3Right: The position that the robot should be at to be on the right side of side 3 on the reef.
frc.robot.Constants.Poses.REEF_Side4Left:  The position that the robot should be at to be on the left side of side 4 on the reef.
frc.robot.Constants.Poses.REEF_Side4Right: The position that the robot should be at to be on the right side of side 4 on the reef.
frc.robot.Constants.Poses.REEF_Side5Left:  The position that the robot should be at to be on the left side of side 5 on the reef.
frc.robot.Constants.Poses.REEF_Side5Right: The position that the robot should be at to be on the right side of side 5 on the reef.
frc.robot.Constants.Poses.REEF_Side6Left:  The position that the robot should be at to be on the left side of side 6 on the reef.
frc.robot.Constants.Poses.REEF_Side6Right: The position that the robot should be at to be on the right side of side 6 on the reef.