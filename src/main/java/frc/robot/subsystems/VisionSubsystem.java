package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Limelight;
import frc.lib.Telemetry;
import frc.robot.Constants.LL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;

public class VisionSubsystem extends SubsystemBase{
    private Limelight centerLimelight;
    private Limelight leftLimelight;
    private Limelight rightLimelight;
    private Limelight[] limelights;

    public VisionSubsystem() {
        centerLimelight = new Limelight(LL.centerLLNT, new Pose3d(), 0.2);
        leftLimelight = new Limelight(LL.leftLLNT,  LL.leftLLOffsetMeters, 0.4);
        rightLimelight = new Limelight(LL.rightLLNT, LL.rightLLOffsetMeters, 0.4);

        centerLimelight.setPipelineIndex(LL.apriltagPipelineIndex);

        leftLimelight.setPipelineIndex(LL.apriltagPipelineIndex);
        rightLimelight.setPipelineIndex(LL.apriltagPipelineIndex);

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
        if ( centerLimelight.hasTargetDebounced()  && centerLimelight.getPipeLineIndex() == LL.apriltagPipelineIndex ) {
            System.out.println("MY FATHER 1" + Timer.getFPGATimestamp());
            poseEstimator.addVisionMeasurement(
                centerLimelight.getPose(), 
                Timer.getFPGATimestamp() - centerLimelight.getLatency(),
            createVisionVector( centerLimelight ) );
        }

        if ( leftLimelight.hasTargetDebounced()  && leftLimelight.getPipeLineIndex() == LL.apriltagPipelineIndex ) {
            System.out.println("MY FATHER 2" + Timer.getFPGATimestamp());
            poseEstimator.addVisionMeasurement(
                leftLimelight.getPose(), 
                Timer.getFPGATimestamp() - leftLimelight.getLatency(),
            createVisionVector( leftLimelight ) );
        }

        if ( rightLimelight.hasTargetDebounced() && rightLimelight.getPipeLineIndex() == LL.apriltagPipelineIndex ) {
            System.out.println("MY FATHER 3" + Timer.getFPGATimestamp());
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

        Telemetry.setValue("RLIMELIGHT/XLEFT", centerLimelight.getPose().minus(leftLimelight.getPose()).getTranslation().getX());
        Telemetry.setValue("RLIMELIGHT/XRIGHT", centerLimelight.getPose().minus(rightLimelight.getPose()).getTranslation().getX());
        Telemetry.setValue("RLIMELIGHT/YLEFT", centerLimelight.getPose().minus(leftLimelight.getPose()).getTranslation().getY());
        Telemetry.setValue("RLIMELIGHT/YRIGHT", centerLimelight.getPose().minus(rightLimelight.getPose()).getTranslation().getY());
    }
}
