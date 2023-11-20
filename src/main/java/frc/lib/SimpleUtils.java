package frc.lib;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DRIVETRAIN;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SimpleUtils {
    public static ChassisSpeeds addChassisSpeeds(ChassisSpeeds speeds1, ChassisSpeeds speeds2) {
        return new ChassisSpeeds(
            speeds1.vxMetersPerSecond + speeds2.vxMetersPerSecond, 
            speeds1.vyMetersPerSecond + speeds2.vyMetersPerSecond, 
            speeds1.omegaRadiansPerSecond + speeds2.omegaRadiansPerSecond);
    }

    public static void chassisSpeedsToTelemetry(String key, ChassisSpeeds speeds) {
        Telemetry.setValue(key + "/XOUTPUT", speeds.vxMetersPerSecond);
        Telemetry.setValue(key + "/YOUTPUT", speeds.vyMetersPerSecond);
        Telemetry.setValue(key + "/OMEGAOUTPUT", speeds.omegaRadiansPerSecond);
    }

    public static void poseToTelemetry(Pose2d pose, String key) {
        Telemetry.setValue("drivetrain/" + key + "/x", pose.getTranslation().getX());
        Telemetry.setValue("drivetrain/" + key + "/y", pose.getTranslation().getY());
        Telemetry.setValue("drivetrain/" + key + "/heading", pose.getRotation().getDegrees());
      }

    public static double degreesToFalcon(double degrees) {
        return degrees / (360.0 / ( DRIVETRAIN.AZIMUTH_GEAR_RATIO * 2048.0));
    }

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
}
