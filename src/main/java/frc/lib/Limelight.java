package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
// import frc.robot.RobotContainer;

public class Limelight {
  private NetworkTable limelight;
  private int pipelineIndex;
  private double[] posevalues;
  private Pose3d offset;
  private String key;

  public Limelight(String key, Pose3d offset) {
    this.key = key;
    limelight = NetworkTableInstance.getDefault().getTable(key);
    // limelight.getEntry("pipeline").setNumber(1);
    this.offset = offset;
    setPipelineIndex(1);
  }

  public void setPipelineIndex(int index) {
    limelight.getEntry("getpipe").setNumber(index);
    limelight.getEntry("pipeline").setNumber(index);
  }

  public int getPipeLineIndex() {
    return pipelineIndex;
  }

  public boolean hasTarget() {
    return ( limelight.getEntry("tv").getDouble(0) == 1 );
  }

  public String getObjectType() {
    return limelight.getEntry("tclass").getString("cube");
  }

  public double getYaw() {
    return limelight.getEntry("tx").getDouble(0);
  }

  public double getPitch() {
    return limelight.getEntry("ty").getDouble(0);
  }

  public double getArea() {
    return limelight.getEntry("ta").getDouble(0);
  }

  public Pose2d getPose() {
    posevalues = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

    Translation2d translate = new Translation2d(posevalues[0], posevalues[1]);
    Rotation2d rotation = Rotation2d.fromDegrees(posevalues[3]);
    
    return new Pose2d(translate, rotation);
  }

  public Pose2d getTarget() {
    posevalues = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    Translation2d translate = new Translation2d(posevalues[0] - offset.getX(), posevalues[1] - offset.getY());
    Rotation2d rotation = new Rotation2d(Math.toRadians(posevalues[3]) - offset.getRotation().getX());
    return new Pose2d(translate, rotation);
  }

  /** returns latency in seconds (tl + cl) */
  public double getLatency () {
    return (limelight.getEntry("tl").getDouble(0) + limelight.getEntry("cl").getDouble(0))/1000.0;
  }

  public void periodic() {
    // limelight.getEntry("getpipe").setNumber(pipelineIndex);

    frc.lib.Telemetry.setValue("R"+key+"/2d/yaw", getYaw());
    frc.lib.Telemetry.setValue("R"+key+"/2d/pitch", getPitch());
    frc.lib.Telemetry.setValue("R"+key+"/2d/area", getArea());
    frc.lib.Telemetry.setValue("R"+key+"/2d/clas", getObjectType());
    frc.lib.Telemetry.setValue("R"+key+"/pip/pipeline", getPipeLineIndex());
    frc.lib.Telemetry.setValue("R"+key+"/hastarget", hasTarget());
    frc.lib.Telemetry.setValue("R"+key+"/Odometry/X", getPose().getX());
    frc.lib.Telemetry.setValue("R"+key+"/Odometry/Y", getPose().getY());
    frc.lib.Telemetry.setValue("R"+key+"/Odometry/Rotation", getPose().getRotation().getDegrees());
    frc.lib.Telemetry.setValue("R"+key+"/Odometry/Norm", getTarget().getTranslation().getNorm());

  }
}