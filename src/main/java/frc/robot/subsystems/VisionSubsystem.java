package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Limelight;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

public class VisionSubsystem extends SubsystemBase{
    private Limelight centerLimelight;
    private Limelight leftLimelight;
    private Limelight rightLimelight;
    private Debouncer deb;
    private Limelight[] limelights;

    public VisionSubsystem() {
        centerLimelight = new Limelight("limelight-limeone", new Pose3d());
        // leftLimelight = new Limelight("limelight-left", 
        //     new Pose3d( 
        //         new Translation3d(0.13, 0.09, 0.0),
        //         new Rotation3d(Math.toRadians(43), 0, 0)));
        // leftLimelight = new Limelight("limelight-left", 
        //     new Pose3d( 
        //         new Translation3d(-0.13, 0.09, 0.0),
        //         new Rotation3d(Math.toRadians(43), 0, 0)));
        centerLimelight.setPipelineIndex(1);
        deb = new Debouncer(0.2);

        limelights = new Limelight[] {
            // leftLimelight,
            // rightLimelight,
            centerLimelight
        };
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
            createVisionVector(getCenterLimelight()) );
        }

        // for(int i = 0; i < limelights.length; i++) {
        //     if ( deb.calculate( getCenterLimelight().hasTarget() ) ) {
        //         System.out.println("MY FATHER" + Timer.getFPGATimestamp());
        //         poseEstimator.addVisionMeasurement(
        //             limelights[i].getPose(), 
        //             Timer.getFPGATimestamp() - limelights[i].getLatency(),
        //         createVisionVector() );
        //     }
        // }
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
    }
}
