package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.AdjustableValues;
import frc.robot.util.MathUtils;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
    public static Command drive(Drive drivetrain, Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> zSteerSupplier, Supplier<Boolean> fieldRelative) {
        return Commands.run(() -> {
            // Getting values from the suppliers.
            // Applying a deadband with an offset and then letting the normal values come through at an input of 0.9.
            double xSpeed = MathUtils.applyDeadbandWithOffsets(xSpeedSupplier.get(), 0.1, 0.9);
            double ySpeed = MathUtils.applyDeadbandWithOffsets(ySpeedSupplier.get(), 0.1, 0.9);
            double zSteer = MathUtils.applyDeadbandWithOffsets(zSteerSupplier.get(), 0.1, 0.9);

            // Applying max speeds
            xSpeed *= AdjustableValues.getNumber("DriveX_Percent");
            ySpeed *= AdjustableValues.getNumber("DriveY_Percent");
            zSteer *= AdjustableValues.getNumber("Steer_Percent");

            // Getting speeds
            ChassisSpeeds speeds = new ChassisSpeeds(
                DriveConstants.maxLinearVelocity.times(-xSpeed),
                DriveConstants.maxLinearVelocity.times(-ySpeed),
                DriveConstants.maxAngularVelocity.times(-zSteer));
            
            // Checking whether to drive field relative or not.
            if (fieldRelative.get()) {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drivetrain.getPose().getRotation());
            }

            // Driving the robot
            drivetrain.drive(speeds);
        }, drivetrain);
    }

    public static Command pathfindToNearestPose(Drive drivetrain, Pose2d... poses) {
        return pathfindToNearestPose(drivetrain, Arrays.asList(poses));
    }

    public static Command pathfindToNearestPose(Drive drivetrain, List<Pose2d> poses) {
        Pose2d endPose = drivetrain.getPose().nearest(poses);
        Logger.recordOutput("/Drive/PIDPose", endPose);

        var wrapper = new Object() {
            public int shouldEnds = 0;
        };

        return Commands.runEnd(() -> {
            Pose2d curPose = drivetrain.getPose();

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                drivetrain.xController.calculate(curPose.getX(), endPose.getX()),
                drivetrain.yController.calculate(curPose.getY(), endPose.getY()),
                drivetrain.thetaController.calculate(curPose.getRotation().getRadians(), endPose.getRotation().getRadians()),
                curPose.getRotation());
    
            drivetrain.drive(speeds);
            
        }, () -> {
            drivetrain.drive(new ChassisSpeeds());
        }, drivetrain).until(() -> {
            boolean shouldEnd = drivetrain.xController.atSetpoint() && drivetrain.yController.atSetpoint() && drivetrain.thetaController.atSetpoint();

            if (shouldEnd) {
                wrapper.shouldEnds++;
            } else {
                wrapper.shouldEnds = 0;
            }

            return wrapper.shouldEnds > 5;
        });
    }

    public static Command xStates(Drive drivetrain) {
        return Commands.run(drivetrain::xStates, drivetrain);
    }
}