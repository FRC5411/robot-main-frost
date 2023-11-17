package frc.lib;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
}
