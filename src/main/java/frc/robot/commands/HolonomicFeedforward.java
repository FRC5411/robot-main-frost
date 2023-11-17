package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicFeedforward {
    public FFConstants xConstants;
    public FFConstants yConstants;
    public FFConstants thetaConstants;

    public HolonomicFeedforward(FFConstants xConstants, FFConstants yConstants, FFConstants thetaConstants) {
        this.xConstants = xConstants;
        this.yConstants = yConstants;
        this.thetaConstants = thetaConstants;
    }

    public ChassisSpeeds calculate(ChassisSpeeds velocity, Transform2d error) {
        return new ChassisSpeeds(
            xConstants.calculate(velocity.vxMetersPerSecond, error.getX()), 
            yConstants.calculate(velocity.vyMetersPerSecond, error.getY()), 
            thetaConstants.calculate(velocity.omegaRadiansPerSecond, error.getRotation().getRotations()));
    }
    
    public static class FFConstants {
        public double kS, kG, kV;

        public FFConstants(double kS, double kG, double kV) {
            this.kS = kS;
            this.kG = kG;
            this.kV = kV;
        }

        public double calculate(double velocity, double error) {
            return kS * Math.signum(velocity) + kG * Math.signum(error) + kV * velocity;
        }
    }
}
