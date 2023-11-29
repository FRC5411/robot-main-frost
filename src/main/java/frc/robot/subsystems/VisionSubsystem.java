package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Limelight;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.numbers.N3;

public class VisionSubsystem extends SubsystemBase{
    private Limelight centerLimelight;
    private Debouncer deb;

    public VisionSubsystem() {
        centerLimelight = new Limelight("limelight-limeone");
        centerLimelight.setPipelineIndex(1);
        deb = new Debouncer(0.2);
    }

    public Limelight getCenterLimelight() {
        return centerLimelight;
    }

    public double getNorm() {
        return getCenterLimelight().getTarget().getTranslation().getNorm();
    }

    public void addVisionMeasurement(SwerveDrivePoseEstimator poseEstimator) {
        if ( deb.calculate( getCenterLimelight().hasTarget() ) ) {
            System.out.println("MY FATHER" + Timer.getFPGATimestamp());
            poseEstimator.addVisionMeasurement(
              getCenterLimelight().getPose(), 
              Timer.getFPGATimestamp() - getCenterLimelight().getLatency(),
              createVisionVector() );
          }
    }

    public Vector<N3> createVisionVector() {
        return VecBuilder.fill(
            3.8 * getNorm(), 
            (getCenterLimelight().getTarget().getX() < 3.46) ? 3.8 * getNorm() : 5.6 * getNorm(), 
            10000000);
    }

    @Override
    public void periodic() {
        centerLimelight.periodic();
    }
}
