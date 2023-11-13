package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.VecBuilder;
import static frc.robot.Constants.LL.*;

import frc.lib.Limelight;

public class VisionSubsystem extends SubsystemBase{
    private Limelight centerLimelight;

    public VisionSubsystem() {
        centerLimelight = new Limelight("limelight-limeone");
    }

    public Limelight getCenterLimelight() {
        return centerLimelight;
    }

    public double getDistanceFromTag() {
        return centerLimelight.getTarget().getTranslation().getNorm();
    }

    public void addVisionMeasurement(SwerveDrivePoseEstimator estimator) {
        if( centerLimelight.hasTarget() ) {
            estimator.addVisionMeasurement(
                centerLimelight.getPose(), 
                Timer.getFPGATimestamp() - centerLimelight.getLatency(),
                VecBuilder.fill(
                    translationGains * getDistanceFromTag(), 
                    translationGains * getDistanceFromTag(), 
                    rotationGains ) );
        }
    }

    @Override
    public void periodic() {
        centerLimelight.periodic();
    }
}
