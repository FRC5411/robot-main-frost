package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Limelight;
import frc.robot.Constants.LL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;

public class VisionSubsystem extends SubsystemBase{
    private Limelight centerLimelight;
    private Limelight leftLimelight;
    private Limelight rightLimelight;
    private Debouncer deb;
    private Limelight[] limelights;

    public VisionSubsystem() {
        centerLimelight = new Limelight(LL.centerLLNT, new Pose3d());
        leftLimelight = new Limelight(LL.leftLLNT,  LL.leftLLOffsetMeters);
        rightLimelight = new Limelight(LL.rightLLNT, LL.rightLLOffsetMeters);
        centerLimelight.setPipelineIndex(LL.gamePiecePipelineIndex);
        leftLimelight.setPipelineIndex(0);
        rightLimelight.setPipelineIndex(0);
        deb = new Debouncer(0.2);

        limelights = new Limelight[] {
            leftLimelight,
            rightLimelight,
            centerLimelight
        };
    }

    public Limelight getCenterLimelight() {
        return centerLimelight;
    }

    public Limelight getLeftLimelight() {
        return leftLimelight;
    }

    public Limelight getRightLimelight() {
        return rightLimelight;
    }

    public double getNorm() {
        return getCenterLimelight().getTarget().getTranslation().getNorm();
    }

    public void setPipelineIndices(int index) {
        for(Limelight cam : limelights) cam.setPipelineIndex(index);
    }

    public void addVisionMeasurement(SwerveDrivePoseEstimator poseEstimator) {
        if ( deb.calculate( centerLimelight.hasTarget() ) ) {
            System.out.println("MY FATHER" + Timer.getFPGATimestamp());
            poseEstimator.addVisionMeasurement(
                centerLimelight.getPose(), 
                Timer.getFPGATimestamp() - centerLimelight.getLatency(),
            createVisionVector( centerLimelight ) );
        }

        if ( deb.calculate( leftLimelight.hasTarget() ) ) {
            System.out.println("MY FATHER" + Timer.getFPGATimestamp());
            poseEstimator.addVisionMeasurement(
                leftLimelight.getPose(), 
                Timer.getFPGATimestamp() - leftLimelight.getLatency(),
            createVisionVector( leftLimelight ) );
        }

        if ( deb.calculate( rightLimelight.hasTarget() ) ) {
            System.out.println("MY FATHER" + Timer.getFPGATimestamp());
            poseEstimator.addVisionMeasurement(
                rightLimelight.getPose(), 
                Timer.getFPGATimestamp() - rightLimelight.getLatency(),
            createVisionVector( rightLimelight ) );
        }
    }

    public Vector<N3> createVisionVector(Limelight cam) {
        return VecBuilder.fill(
            3.8 * getNorm(), 
            (cam.getTarget().getX() < 3.46) ? 3.8 * getNorm() : 5.6 * getNorm(), 
            10000000);
    }

    @Override
    public void periodic() {
        centerLimelight.periodic();
        leftLimelight.periodic();
        rightLimelight.periodic();
    }
}
