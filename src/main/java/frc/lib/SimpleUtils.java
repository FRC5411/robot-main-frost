package frc.lib;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SimpleUtils {
    public static ChassisSpeeds discretize(ChassisSpeeds speeds) {
        double dt = 0.02;
        var desiredDeltaPose = new Pose2d(
          speeds.vxMetersPerSecond * dt, 
          speeds.vyMetersPerSecond * dt, 
          new Rotation2d(speeds.omegaRadiansPerSecond * dt * 3)
        );
        var twist = new Pose2d().log(desiredDeltaPose);
    
        return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
      }
    
    public static void poseToTelemetry(Pose2d pose, String key) {
        Telemetry.setValue("drivetrain/" + key + "/x", pose.getTranslation().getX());
        Telemetry.setValue("drivetrain/" + key + "/y", pose.getTranslation().getY());
        Telemetry.setValue("drivetrain/" + key + "/heading", pose.getRotation().getDegrees());
      }
}
