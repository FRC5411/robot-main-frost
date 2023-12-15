package frc.robot.subsystems;
import static frc.robot.Constants.DRIVETRAIN.*;
import static frc.robot.Constants.CAN.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.FrostConfigs;
import frc.lib.HolonomicController;
import frc.lib.HolonomicFeedforward;
import frc.lib.SimpleUtils;
import frc.lib.SwerveModule;
import frc.lib.Telemetry;
import frc.lib.HolonomicController.HolonomicConstraints;
import frc.lib.HolonomicFeedforward.FFConstants;
import frc.robot.Constants.LL;
import frc.robot.Constants.ARM.positions;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.moveToPosition;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class Drivetrain extends SubsystemBase {
  private Pigeon m_gyro;
  private Arm m_arm;
  private PinchersofPower m_claw;
  private VisionSubsystem vision;

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(  ROBOT_WIDTH_METERS / 2,  ROBOT_WIDTH_METERS / 2 ),
    new Translation2d(  ROBOT_WIDTH_METERS / 2, -ROBOT_WIDTH_METERS / 2 ),
    new Translation2d( -ROBOT_WIDTH_METERS / 2,  ROBOT_WIDTH_METERS / 2 ),
    new Translation2d( -ROBOT_WIDTH_METERS / 2, -ROBOT_WIDTH_METERS / 2 ) );

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds forwardKinematics = new ChassisSpeeds();

  private SwerveModuleState[] modules = new SwerveModuleState[4];

  private final CANCoder FL_Position = new CANCoder(FL_CANCODER_ID, "drivetrain");
  private final CANCoder FR_Position = new CANCoder(FR_CANCODER_ID, "drivetrain");
  private final CANCoder BL_Position = new CANCoder(BL_CANCODER_ID, "drivetrain");
  private final CANCoder BR_Position = new CANCoder(BR_CANCODER_ID, "drivetrain");

  private final TalonFX FL_Drive = new TalonFX(FL_DRIVE_ID, "drivetrain");
  private final TalonFX FR_Drive = new TalonFX(FR_DRIVE_ID, "drivetrain");
  private final TalonFX BL_Drive = new TalonFX(BL_DRIVE_ID, "drivetrain");
  private final TalonFX BR_Drive = new TalonFX(BR_DRIVE_ID, "drivetrain");

  private final CANSparkMax shwerveDrive = new CANSparkMax(SHWERVE_DRIVE_ID, MotorType.kBrushless);

  private final TalonFX FL_Azimuth = new TalonFX(FL_AZIMUTH_ID, "drivetrain");
  private final TalonFX FR_Azimuth = new TalonFX(FR_AZIMUTH_ID, "drivetrain");
  private final TalonFX BL_Azimuth = new TalonFX(BL_AZIMUTH_ID, "drivetrain");
  private final TalonFX BR_Azimuth = new TalonFX(BR_AZIMUTH_ID, "drivetrain");

  private final PIDController FL_PID = new PIDController(0.0094, 0, 0.000277); // 0.105
  private final PIDController FR_PID = new PIDController(0.0095, 0, 0.000270);
  private final PIDController BL_PID = new PIDController(0.0096, 0, 0.000270);
  private final PIDController BR_PID = new PIDController(0.0093, 0, 0.000270);

  private final double FL_kF = 0.047;
  private final double FR_kF = AZIMUTH_kF;
  private final double BL_kF = AZIMUTH_kF;
  private final double BR_kF = 0.05;

  private SwerveModule[] swerveModules = new SwerveModule[4];

  private boolean isRobotOriented = false;

  private static SwerveDrivePoseEstimator m_poseEstimator;
  private static SwerveDriveOdometry m_Odometry;

  private List<Pose2d> _coneWaypoints = new ArrayList<Pose2d>();
  private List<Pose2d> _cubeWaypoints = new ArrayList<Pose2d>();

  private Pose2d _robotPose = new Pose2d();
  private Pose2d _lastPose = _robotPose;

  private double downChargeLine = 1.0;
  private double upChargeLine = 4.9;
  private double rightChargeLine = 14.05;
  private double leftChargeLine = 11.0;

  private double _translationKp = 2.40;// 2.35//1.8;//3.25;//2.75;//2.5;//2.1;//2;//0.018;//0.03;//0.004 0.001
  private double _translationKi = 0;
  private double _translationKd = 0;
  private double _rotationKp = 1.83;//1.83;// 2.5//12.5;//15;//0.00005
  private double _rotationKi = 0;
  private double _rotationKd = 0.085; // 0.1

  private double _alignXTranslationKp = 3.5;//5.5;//5; //5.5;
  private double _alignXTranslationKi = 0.17;//0.1;//0.;
  private double _alignXTranslationKd = 0.018;//0.05;
  private FFConstants xFFConstants = new FFConstants(0, 0.15, 1.25);

  private double _alignYTranslationKp = 2.5;//2.2;//3.1; //5.5;
  private double _alignYTranslationKi = 0.05; //0.01;//0.;
  private double _alignYTranslationKd = 0.02; //0.03;
  private FFConstants yFFConstants = new FFConstants(0.0, 0.0, 1.02);

  private double _alignRotationKp = 6.2;//2.5;
  private double _alignRotationKi = 0.01;// 0.03; //.42;
  private double _alignRotationKd = 0;//.0;
  private FFConstants thetaFFConstants = new FFConstants(3.0, 0.0, 0.0);

  public Field2d field2d = new Field2d();

  private moveToPosition _moveToPosition;
  
  private Constraints _tranYConstraints = new Constraints(4, 6);
  private Constraints _tranXConstraints = new Constraints(3, 5);
  private Constraints _rotConstraints = new Constraints(360, 240);

  private HolonomicConstraints _holonomicConstraints = 
    new HolonomicConstraints(_tranXConstraints, _tranYConstraints, _rotConstraints);

  Pose2d _targetPose = new Pose2d();

  public Drivetrain(Pigeon m_gyro, Arm m_arm, PinchersofPower m_claw, VisionSubsystem vision) {
    this.m_gyro = m_gyro;
    this.m_arm = m_arm;
    this.m_claw = m_claw;
    this.vision = vision;

    FrostConfigs.configPID(FL_PID);
    FrostConfigs.configPID(FR_PID);
    FrostConfigs.configPID(BL_PID);
    FrostConfigs.configPID(BR_PID);

    FrostConfigs.configDrive(FL_Drive);
    FrostConfigs.configDrive(FR_Drive);
    FrostConfigs.configDrive(BL_Drive);
    FrostConfigs.configDrive(BR_Drive);

    FrostConfigs.configPosition(FL_Position, FL_ECODER_OFFSET);
    FrostConfigs.configPosition(FR_Position, FR_ECODER_OFFSET);
    FrostConfigs.configPosition(BL_Position, BL_ECODER_OFFSET);
    FrostConfigs.configPosition(BR_Position, BR_ECODER_OFFSET);

    FrostConfigs.configAzimuth(FL_Azimuth, FL_Position, FL_PID.getP(), FL_PID.getD(), FL_kF);
    FrostConfigs.configAzimuth(FR_Azimuth, FR_Position, FR_PID.getP(), FR_PID.getD(), FR_kF);
    FrostConfigs.configAzimuth(BL_Azimuth, BL_Position, BL_PID.getP(), BL_PID.getD(), BL_kF);
    FrostConfigs.configAzimuth(BR_Azimuth, BR_Position, BR_PID.getP(), BR_PID.getD(), BR_kF);
    
    swerveModules[0] = new SwerveModule( FL_Drive, FL_Azimuth, FL_Position, FL_PID, FL_kF, "FL" );
    swerveModules[1] = new SwerveModule( FR_Drive, FR_Azimuth, FR_Position, FR_PID, FR_kF, "FR" );
    swerveModules[2] = new SwerveModule( BL_Drive, BL_Azimuth, BL_Position, BL_PID, BL_kF, "BL" );
    swerveModules[3] = new SwerveModule( BR_Drive, BR_Azimuth, BR_Position, BR_PID, BR_kF, "BR" );

    FrostConfigs.configShwerve(shwerveDrive);

    m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics, 
      new Rotation2d(0), 
      getSwerveModulePositions(), 
      new Pose2d());

    m_Odometry = new SwerveDriveOdometry(
      m_kinematics, 
      new Rotation2d(0), 
      getSwerveModulePositions(), 
      new Pose2d());

    Telemetry.setValue("drivetrain/PathPlanner/translationKp", _translationKp);
    Telemetry.setValue("drivetrain/PathPlanner/translationKi", _translationKi);
    Telemetry.setValue("drivetrain/PathPlanner/translationKd", _translationKd);
    Telemetry.setValue("drivetrain/PathPlanner/rotationKp", _rotationKp);
    Telemetry.setValue("drivetrain/PathPlanner/rotationKi", _rotationKi);
    Telemetry.setValue("drivetrain/PathPlanner/rotationKd", _rotationKd);;

   if (RobotContainer.getDriverAlliance().equals(DriverStation.Alliance.Red)) {
      _coneWaypoints.add(new Pose2d(0.76, 6.13, Rotation2d.fromDegrees(180)));
      _coneWaypoints.add(new Pose2d(0.76, 7.49, Rotation2d.fromDegrees(180)));
      _coneWaypoints.add(new Pose2d(14.70, 4.98, new Rotation2d()));
      _coneWaypoints.add(new Pose2d(14.70, 3.94 - 0.05, new Rotation2d()));
      _coneWaypoints.add(new Pose2d(14.70, 3.38 - 0.05, new Rotation2d()));
      _coneWaypoints.add(new Pose2d(14.70, 2.28 - 0.05, new Rotation2d()));
      _coneWaypoints.add(new Pose2d(14.70, 1.67, new Rotation2d()));
      _coneWaypoints.add(new Pose2d(14.70, 0.48, new Rotation2d()));

      _cubeWaypoints.add(new Pose2d(0.76, 6.13, Rotation2d.fromDegrees(180)));
      _cubeWaypoints.add(new Pose2d(0.76, 7.49, Rotation2d.fromDegrees(180)));
      _cubeWaypoints.add(new Pose2d(14.70, 1.13 - 0.05, new Rotation2d()));
      _cubeWaypoints.add(new Pose2d(14.70, 2.95 - 0.05, new Rotation2d()));
      _cubeWaypoints.add(new Pose2d(14.70, 4.52 - 0.05, new Rotation2d()));
    } else if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
      _coneWaypoints.add(new Pose2d(15.79, 7.33, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(15.79, 6.00, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.82, 5.05, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.82, 3.84, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.82, 3.28, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.82, 2.18, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.82, 1.60, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.82, 0.47, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(1.82, 1.03, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(1.82, 2.75, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(1.82, 4.42, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(15.79, 7.33, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(15.79, 6.00, new Rotation2d(0)));
    }

    _moveToPosition = new moveToPosition(
      this::getPose,
      this::getChassisSpeeds,
      this::driveFromChassisSpeedsField,
      this,
      this.vision);

    PathPlannerServer.startServer(6969);

    if(vision.getCenterLimelight().hasTarget()) resetPose(vision.getCenterLimelight().getPose());
  }

  @Override
  public void periodic() {
    for(int i = 0; i < swerveModules.length; i++) swerveModules[i].telemetry();

    Telemetry.setValue("drivetrain/isRobotOriented", isRobotOriented);
    robotPositionTelemetry();
  }

  public void robotPositionTelemetry() {
    // vision.addVisionMeasurement(m_poseEstimator);
    if ( vision.getCenterLimelight().hasTargetDebounced()  && vision.getCenterLimelight().getPipeLineIndex() == LL.apriltagPipelineIndex ) {
      System.out.println("MY FATHER 1" + Timer.getFPGATimestamp());
      m_poseEstimator.addVisionMeasurement(
          vision.getCenterLimelight().getPose(), 
          Timer.getFPGATimestamp() - vision.getCenterLimelight().getLatency(),
      vision.createVisionVector( vision.getCenterLimelight() ) );
  }

  if ( vision.getLeftLimelight().hasTargetDebounced()  && vision.getLeftLimelight().getPipeLineIndex() == LL.apriltagPipelineIndex ) {
      System.out.println("MY FATHER 2" + Timer.getFPGATimestamp());
      m_poseEstimator.addVisionMeasurement(
          vision.getLeftLimelight().getPose(), 
          Timer.getFPGATimestamp() - vision.getLeftLimelight().getLatency(),
      vision.createVisionVector( vision.getLeftLimelight() ) );
  }

  if ( vision.getRightLimelight().hasTargetDebounced() && vision.getRightLimelight().getPipeLineIndex() == LL.apriltagPipelineIndex ) {
      System.out.println("MY FATHER 3" + Timer.getFPGATimestamp());
      m_poseEstimator.addVisionMeasurement(
          vision.getRightLimelight().getPose(), 
          Timer.getFPGATimestamp() - vision.getRightLimelight().getLatency(),
      vision.createVisionVector( vision.getRightLimelight() ) );
  }

    // System.out.println("MY MOTHER" + Timer.getFPGATimestamp());
    _robotPose = m_poseEstimator.update(new Rotation2d(Math.toRadians(m_gyro.getYaw())), getSwerveModulePositions());

    Transform2d transform = _robotPose.minus(_lastPose).div(0.02);

    forwardKinematics = new ChassisSpeeds(
      transform.getX(), 
      transform.getY(), 
      transform.getRotation().getDegrees() );

    SimpleUtils.poseToTelemetry( _robotPose, "/chassis/" );
    SimpleUtils.poseToTelemetry(m_Odometry.update(new Rotation2d(Math.toRadians(m_gyro.getYaw())), getSwerveModulePositions()), "/chassisOdometry/");
    Telemetry.setValue("drivetrain/chassis/robot/forwardSpeed", forwardKinematics.vxMetersPerSecond);
    Telemetry.setValue("drivetrain/chassis/robot/rightwardSpeed", forwardKinematics.vyMetersPerSecond);
    Telemetry.setValue("drivetrain/chassis/clockwiseSpeed", Math.toDegrees(forwardKinematics.omegaRadiansPerSecond));

    _lastPose = _robotPose;

    field2d.setRobotPose(_robotPose);
    // field2d.getObject("Pure Odometry").setPose(m_Odometry.getPoseMeters());
    // field2d.getObject("Pure Camera Center").setPose(vision.getCenterLimelight().getPose());
    // field2d.getObject("Pure Camera Left").setPose(vision.getLeftLimelight().getPose());
    // field2d.getObject("Pure Camera Right").setPose(vision.getRightLimelight().getPose());
    SmartDashboard.putData(field2d);    
  }

  public void joystickDrive(double LX, double LY, double RX) {
    if ( !isRobotOriented ) m_chassisSpeeds = 
      ChassisSpeeds.fromFieldRelativeSpeeds(
        LY * MAX_LINEAR_SPEED,
        -LX * MAX_LINEAR_SPEED,
        -RX * MAX_ROTATION_SPEED, 
        m_poseEstimator.getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(
          (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) ? 180 : 0 ) ) );

    else m_chassisSpeeds = new ChassisSpeeds(LY * MAX_LINEAR_SPEED, -LX * MAX_LINEAR_SPEED, -RX * MAX_ROTATION_SPEED);

    m_chassisSpeeds = SimpleUtils.discretize( m_chassisSpeeds );
    modules = m_kinematics.toSwerveModuleStates( m_chassisSpeeds );

    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);
    setDesiredStates();
  }

  // For autonomous
  public void driveFromModuleStates ( SwerveModuleState[] modules ) {
    modules = m_kinematics.toSwerveModuleStates( SimpleUtils.discretize( m_kinematics.toChassisSpeeds(modules) ) );
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);
    setDesiredStates();
  }

  public void driveFromChassisSpeeds (ChassisSpeeds speeds) {
    ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds, m_poseEstimator.getEstimatedPosition().getRotation() );

    modules = m_kinematics.toSwerveModuleStates( speeds );
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);
    setDesiredStates();
  }

  public void driveFromChassisSpeedsField (ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds, m_poseEstimator.getEstimatedPosition().getRotation() );

    modules = m_kinematics.toSwerveModuleStates( speeds );
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);
    setDesiredStates();
  }

  public void driveFromChassisSpeedsLocked (ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds, m_poseEstimator.getEstimatedPosition().getRotation() );

    modules = m_kinematics.toSwerveModuleStates( speeds );
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);
    setLockedStates();
  }

  public void stopModules () { for(int i = 0; i <= 3; i++) swerveModules[i].stopMotors(); }

  public void setDesiredStates() { for(int i = 0; i <= 3; i++) swerveModules[i].setDesiredState( modules[i] ); }

  public void setLockedStates() { for(int i = 0; i <= 3; i++) swerveModules[i].setLockedState( modules[i] ); }

  public void shwerve ( double LX, double LY) {
    // 6in diameter wheels, 10:1 o
    if (isRobotOriented) shwerveDrive.set(MathUtil.clamp(-LX*9, -1, 1));
    else noShwerve();
  }

  public void noShwerve () {
    shwerveDrive.set(0);
  }

  public List<Pose2d> optimizeWaypoints(Pose2d target) {
    List<Pose2d> waypoints = new ArrayList<Pose2d>();
    if(_robotPose.getY() < downChargeLine) waypoints.add( linearOptimize( target, downChargeLine ) );
    else if(_robotPose.getY() > upChargeLine) waypoints.add( linearOptimize( target,  upChargeLine  ) );
    else if(_robotPose.getY() > downChargeLine && _robotPose.getY() < upChargeLine) {
      List<Pose2d> onTheWay = new ArrayList<Pose2d>();
      onTheWay.add( new Pose2d( _robotPose.getX(),   upChargeLine, new Rotation2d() ) );
      onTheWay.add( new Pose2d( _robotPose.getX(), downChargeLine, new Rotation2d() ) );
      Pose2d nearest = _robotPose.nearest( onTheWay );

      // Slope point form
      double a = _robotPose.getY();
      double b = _robotPose.getX();
      double m = (a - target.getY()) / (b - target.getX());
      double xIntersection = m * (nearest.getY() - b) + a;
      double yIntersection = ( (rightChargeLine - a) / m ) + b;

      if(target.getY() > downChargeLine && target.getY() < upChargeLine) {
        waypoints.add( nearest );
        waypoints.add( new Pose2d(14.05, nearest.getY(), new Rotation2d() ) );
        waypoints.add( new Pose2d(14.05, target.getY(), new Rotation2d() ) );
      } else if((xIntersection > leftChargeLine && xIntersection < rightChargeLine) ||
                (yIntersection > downChargeLine && yIntersection < upChargeLine   )) waypoints.add( nearest );
    }
    waypoints.add(new Pose2d(14.05, target.getY(), new Rotation2d()));
    waypoints.add(target);

    return waypoints;
  }

  public Pose2d linearOptimize(Pose2d target, double avoidanceLine) {
    // Uses point slope form to find the equation of the line between the robot and the target
    double a = _robotPose.getY();
    double b = _robotPose.getX();
    double slope = (a - target.getY()) / (b - target.getX());
    double intersection = slope * (avoidanceLine - b) + a;

    if(intersection < 14.05) return (new Pose2d(14.05, avoidanceLine, new Rotation2d(0)));
    return new Pose2d(14.05, target.getY(), target.getRotation());
  }

  public Command moveToPositionCommand () {
    Pose2d actualPose = _robotPose; 

    Pose2d closest = actualPose.nearest( m_claw.wantCone() ? _coneWaypoints : _cubeWaypoints );
    if (closest == null) return new InstantCommand();

    SimpleUtils.poseToTelemetry(actualPose, "Align/startPose");
    SimpleUtils.poseToTelemetry(closest, "Align/choosenWaypoint");
    if(_robotPose.getX() >= 14.05 || _robotPose.getX() < 8) return pathToCommand(closest);
    return pathToCommand( optimizeWaypoints( closest ) );
  }

  public Command pathToCommand (Pose2d target) {
    Command toAlign = _moveToPosition.generateMoveToPositionCommandTimed(
      new Pose2d(
        m_poseEstimator.getEstimatedPosition().getX(), 
        target.getY(), 
        target.getRotation() ),
      new Pose2d( 0.1, 0.1, Rotation2d.fromDegrees(3) ),
      _holonomicConstraints,
      generateAlignmentController() );

    Command toGoal = _moveToPosition.generateMoveToPositionCommand( 
      target,
      new Pose2d(), 
      generateAlignmentController() );

    return new SequentialCommandGroup(toAlign, toGoal);
  }

  public Command pathToCommand(List<Pose2d> waypoints) {
    field2d.getObject("Targets").setPoses(waypoints);
    SequentialCommandGroup commands = new SequentialCommandGroup();

    for(int i = 0; i < waypoints.size() - 1; i++) {
      ChassisSpeeds speeds;
      Transform2d transform = waypoints.get( i ).minus( waypoints.get( i + 1 ) );
      if( !(transform.getX() < 1e-6 && transform.getY() < 1e-6) ) {
        if(transform.getX() < 1e-6) 
          speeds = new ChassisSpeeds(0.25, 0.0, 0.0);
        else if(transform.getY() < 1e-6) 
          speeds = new ChassisSpeeds(0.0, 0.25, 0.0);
        else speeds = new ChassisSpeeds();
      } else speeds = new ChassisSpeeds();

      commands.addCommands(
        _moveToPosition.generateMoveToPositionCommandTimed(
          waypoints.get( i ),
          speeds,
          new Pose2d( 0.1, 0.1, Rotation2d.fromDegrees(3) ),
          _holonomicConstraints,
          generateAlignmentController() ) );
    }

    commands.addCommands(
      _moveToPosition.generateMoveToPositionCommand(
        waypoints.get( waypoints.size() - 1 ),
        new Pose2d(), 
        generateAlignmentController() ));

    return commands;
  }

  public HolonomicController generateAlignmentController() {
    HolonomicController controller = new HolonomicController(
      new ProfiledPIDController(
        _alignXTranslationKp, 
        _alignXTranslationKi,
        _alignXTranslationKd,
        _tranXConstraints), 
      new ProfiledPIDController(
        _alignYTranslationKp, 
        _alignYTranslationKi,
        _alignYTranslationKd,
        _tranYConstraints), 
      new ProfiledPIDController(
        _alignRotationKp, 
        _alignRotationKi,
        _alignRotationKd,
        _rotConstraints),
      new HolonomicFeedforward(
        yFFConstants, 
        xFFConstants, 
        thetaFFConstants));
    
    // controller.xControllerIRange(-0.75, 0.75);
    // controller.yControllerIRange(-0.5, 0.5);
    // controller.thetaControllerIRange(-8.5, 8.5);

    return controller;
  }


  public double getGyroAngle() { return m_gyro.getPitch(); }

  public void zeroGyro() { m_gyro.zeroYaw(); }

  public boolean toggleRobotOrient() { return isRobotOriented = !isRobotOriented; }

  public boolean getIsRobotOriented() { return isRobotOriented; }

  public void setRobotOriented(boolean _isRobotOriented) { isRobotOriented = _isRobotOriented; }

  public Pose2d getPose() { return _robotPose; }

  public void resetPoseWithLL() { 
    resetPose(
      new Pose2d(
        vision.getCenterLimelight().getPose().getX(), 
        vision.getCenterLimelight().getPose().getY(), 
        Rotation2d.fromDegrees( m_gyro.getYaw() ) ) );
      }

  public ChassisSpeeds getChassisSpeeds() { return forwardKinematics; }

  public SwerveDriveKinematics getKinematics() { return m_kinematics; }

  public Pose2d getTargetPose() { return _moveToPosition.getTarget(); }

  private SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(int i = 0; i <= 3; i++) positions[i] = swerveModules[i].getPosition();
    return positions;
  }

  public Command getAutonomousCommand () {
    if (Telemetry.getValue("general/autonomous/selectedRoutine", "dontMove").equals("special")) {
      return new InstantCommand(()->setRobotOriented(true)).andThen(new RepeatCommand(new InstantCommand(()->joystickDrive(0, 0.5, 0))).withTimeout(1).andThen(new InstantCommand(()->stopModules())));
    }

    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    try {
      List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
        Telemetry.getValue("general/autonomous/selectedRoutine", "dontMove"),
        PathPlanner.getConstraintsFromPath(
          Telemetry.getValue("general/autonomous/selectedRoutine", "Mobility")));

      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("marker1", new PrintCommand("Passed marker 1"));
      eventMap.put("placeHighCone", m_arm.goToScoreHigh().withTimeout(1.5));
      eventMap.put("placeMidCone", m_arm.goToScoreMid().withTimeout(1.5));
      eventMap.put("placeHighCube", m_arm.moveToPositionTerminatingCommand(positions.ScoreHighCube).withTimeout(1.5));
      eventMap.put("tuck", m_arm.moveToPositionTerminatingCommand(positions.Idle).withTimeout(0.5));
      eventMap.put("release", m_claw.outTakeCommand().andThen(new WaitCommand(.25)));
      eventMap.put("pickupLow", m_arm.moveToPositionCommand(positions.AutonFloor).withTimeout(0.1));
      eventMap.put("pickupLowAlt", m_arm.moveToPositionCommand(positions.FloorAlt).withTimeout(0.85));
      eventMap.put("intake",(m_claw.intakeCommand().repeatedly().withTimeout(0.5)));
      eventMap.put("autobalance", new AutoBalance(this));
      eventMap.put("coneMode", new InstantCommand( () -> { m_claw.setCone(true); m_claw.closeGrip(); m_claw.spinSlow(); } ));
      eventMap.put("cubeMode", new InstantCommand( () -> { m_claw.setCone(false); m_claw.openGrip(); } ));
      eventMap.put("wait", new WaitCommand(0.75));

      SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        () -> m_poseEstimator.getEstimatedPosition(),
        this::resetPose,
        m_kinematics,
        new PIDConstants(_translationKp, _translationKi, _translationKd),
        new PIDConstants(_rotationKp, _rotationKi, _rotationKd),
        this::driveFromModuleStates,
        eventMap,
        true,
        this
      );

      return autoBuilder.fullAuto(pathGroup);
    } catch (Exception e) {
      // uh oh
      DriverStation.reportError("it crashed LOL " + e.getLocalizedMessage(), true);

      // score a preloaded cone if the auton crashes
      return new SequentialCommandGroup(
        new InstantCommand( () -> stopModules() ),
        new InstantCommand( () -> { m_claw.setCone(true); m_claw.closeGrip(); } ),
        m_arm.moveToPositionTerminatingCommand(positions.ScoreHighCone).withTimeout(2.75).andThen(m_arm.moveToPositionCommand(positions.DipHighCone).withTimeout(0.75)),
        m_claw.outTakeCommand().andThen(new WaitCommand(.25)),
        m_arm.moveToPositionTerminatingCommand(positions.Idle) );
    }
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
      Rotation2d.fromDegrees( m_gyro.getYaw() ), 
      getSwerveModulePositions(), 
      pose);

    m_Odometry.resetPosition(
      Rotation2d.fromDegrees( m_gyro.getYaw() ), 
      getSwerveModulePositions(), 
      pose);
  }
}