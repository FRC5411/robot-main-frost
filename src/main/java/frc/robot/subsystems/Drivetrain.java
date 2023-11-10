package frc.robot.subsystems;
import static frc.robot.Constants.DRIVETRAIN.*;
import static frc.robot.Constants.CAN.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.lib.SwerveModule;
import frc.lib.Telemetry;

import frc.robot.Constants.ARM.positions;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.HolonomicController;
import frc.robot.commands.moveToPosition;
import frc.robot.commands.HolonomicController.HolonomicConstraints;
import frc.robot.RobotContainer;

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
  private Debouncer deb = new Debouncer(0.2);
  
  private static final StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = 
    new StatorCurrentLimitConfiguration(
      true, 
      60, 
      60, 
      0);

  private static final StatorCurrentLimitConfiguration AZIMUTH_CURRENT_LIMIT = 
    new StatorCurrentLimitConfiguration(
      true, 
      30, 
      40, 
      0.2);

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

  private double _alignXTranslationKp = 3.1;//5.5;//5; //5.5;
  private double _alignXTranslationKi = 0.17;//0.1;//0.;
  private double _alignXTranslationKd = 0.018;//0.05;

  private double _alignYTranslationKp = 2.5;//2.2;//3.1; //5.5;
  private double _alignYTranslationKi = 0.05; //0.01;//0.;
  private double _alignYTranslationKd = 0.02; //0.03;

  private double _alignRotationKp = 6.2;//2.5;
  private double _alignRotationKi = 0.01;// 0.03; //.42;
  private double _alignRotationKd = 0;//.0;

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

    configPID(FL_PID);
    configPID(FR_PID);
    configPID(BL_PID);
    configPID(BR_PID);

    configDrive(FL_Drive);
    configDrive(FR_Drive);
    configDrive(BL_Drive);
    configDrive(BR_Drive);

    configPosition(FL_Position, FL_ECODER_OFFSET);
    configPosition(FR_Position, FR_ECODER_OFFSET);
    configPosition(BL_Position, BL_ECODER_OFFSET);
    configPosition(BR_Position, BR_ECODER_OFFSET);

    configAzimuth(FL_Azimuth, FL_Position, FL_PID.getP(), FL_PID.getD(), FL_kF);
    configAzimuth(FR_Azimuth, FR_Position, FR_PID.getP(), FR_PID.getD(), FR_kF);
    configAzimuth(BL_Azimuth, BL_Position, BL_PID.getP(), BL_PID.getD(), BL_kF);
    configAzimuth(BR_Azimuth, BR_Position, BR_PID.getP(), BR_PID.getD(), BR_kF);

    swerveModules[0] = new SwerveModule( FL_Drive, FL_Azimuth, FL_Position, FL_PID, FL_kF, "FL" );
    swerveModules[1] = new SwerveModule( FR_Drive, FR_Azimuth, FR_Position, FR_PID, FR_kF, "FR" );
    swerveModules[2] = new SwerveModule( BL_Drive, BL_Azimuth, BL_Position, BL_PID, BL_kF, "BL" );
    swerveModules[3] = new SwerveModule( BR_Drive, BR_Azimuth, BR_Position, BR_PID, BR_kF, "BR" );

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
    Telemetry.setValue("drivetrain/PathPlanner/rotationKd", _rotationKd);

    shwerveDrive.restoreFactoryDefaults();
    shwerveDrive.clearFaults();
    shwerveDrive.setSmartCurrentLimit(60);
    shwerveDrive.setSecondaryCurrentLimit(60);
    shwerveDrive.burnFlash();

   if (RobotContainer.getDriverAlliance().equals(DriverStation.Alliance.Red)) {
      _coneWaypoints.add(new Pose2d(0.76, 6.13, Rotation2d.fromDegrees(180)));
      _coneWaypoints.add(new Pose2d(0.76, 7.49, Rotation2d.fromDegrees(180)));
      _coneWaypoints.add(new Pose2d(14.80, 4.98, new Rotation2d()));
      _coneWaypoints.add(new Pose2d(14.80, 3.94 - 0.05, new Rotation2d()));
      _coneWaypoints.add(new Pose2d(14.80, 3.38 - 0.05, new Rotation2d()));
      _coneWaypoints.add(new Pose2d(14.80, 2.28 - 0.05, new Rotation2d()));
      _coneWaypoints.add(new Pose2d(14.80, 1.67, new Rotation2d()));
      _coneWaypoints.add(new Pose2d(14.80, 0.48, new Rotation2d()));

      _cubeWaypoints.add(new Pose2d(0.76, 6.13, Rotation2d.fromDegrees(180)));
      _cubeWaypoints.add(new Pose2d(0.76, 7.49, Rotation2d.fromDegrees(180)));
      _cubeWaypoints.add(new Pose2d(14.80, 1.13 - 0.05, new Rotation2d()));
      _cubeWaypoints.add(new Pose2d(14.80, 2.95 - 0.05, new Rotation2d()));
      _cubeWaypoints.add(new Pose2d(14.80, 4.52 - 0.05, new Rotation2d()));
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
      this::driveFromChassisSpeeds,
      this );

    PathPlannerServer.startServer(6969);

    if(vision.getCenterLimelight().hasTarget()) resetPose(vision.getCenterLimelight().getPose());
  }

  @Override
  public void periodic() {
    for(int i = 0; i < swerveModules.length; i++) swerveModules[i].telemetry();

    Telemetry.setValue("drivetrain/isRobotOriented", isRobotOriented);
    // Telemetry.setValue("e", modules);
    robotPositionTelemetry();

    // double[] e = new double[8];
    // e[0] = modules[0].angle.getRadians();
    // e[1] = modules[0].speedMetersPerSecond;
    // e[2] = modules[1].angle.getRadians();
    // e[3] = modules[2].speedMetersPerSecond;
    // SmartDashboard.putNumberArray("e", e);
  }

  public void robotPositionTelemetry() {
    if ( deb.calculate( vision.getCenterLimelight().hasTarget() ) ) {
      System.out.println("MY FATHER" + Timer.getFPGATimestamp());
      m_poseEstimator.addVisionMeasurement(
        vision.getCenterLimelight().getPose(), 
        Timer.getFPGATimestamp() - vision.getCenterLimelight().getLatency(),
        VecBuilder.fill(
          3.8 * vision.getCenterLimelight().getTarget().getTranslation().getNorm(), 
          3.8 * vision.getCenterLimelight().getTarget().getTranslation().getNorm(), 10000000) );
    }

    // System.out.println("MY MOTHER" + Timer.getFPGATimestamp());
    _robotPose = m_poseEstimator.update(new Rotation2d(Math.toRadians(m_gyro.getYaw())), getSwerveModulePositions());

    Transform2d transform = _robotPose.minus(_lastPose).div(0.02);

    forwardKinematics = new ChassisSpeeds(
      transform.getX(), 
      transform.getY(), 
      transform.getRotation().getDegrees() );

    poseToTelemetry( _robotPose, "/chassis/" );
    poseToTelemetry(m_Odometry.update(new Rotation2d(Math.toRadians(m_gyro.getYaw())), getSwerveModulePositions()), "/chassisOdometry/");
    Telemetry.setValue("drivetrain/chassis/robot/forwardSpeed", forwardKinematics.vxMetersPerSecond);
    Telemetry.setValue("drivetrain/chassis/robot/rightwardSpeed", forwardKinematics.vyMetersPerSecond);
    Telemetry.setValue("drivetrain/chassis/clockwiseSpeed", Math.toDegrees(forwardKinematics.omegaRadiansPerSecond));

    _lastPose = _robotPose;

    field2d.setRobotPose(_robotPose);
    field2d.getObject("Pure Odometry").setPose(m_Odometry.getPoseMeters());
    field2d.getObject("Pure Camera").setPose(vision.getCenterLimelight().getPose());
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

    m_chassisSpeeds = discretize( m_chassisSpeeds );
    modules = m_kinematics.toSwerveModuleStates( m_chassisSpeeds );

    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);
    setDesiredStates();
  }

  // For autonomous
  public void driveFromModuleStates ( SwerveModuleState[] modules ) {
    modules = m_kinematics.toSwerveModuleStates( discretize( m_kinematics.toChassisSpeeds(modules) ) );
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);
    setDesiredStates();
  }

  public void driveFromChassisSpeeds (ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds, m_poseEstimator.getEstimatedPosition().getRotation() );

    modules = m_kinematics.toSwerveModuleStates( speeds );
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);
    setDesiredStates();
  }

  public void stopModules () { for(int i = 0; i <= 3; i++) swerveModules[i].stopMotors(); }

  public void setDesiredStates() { for(int i = 0; i <= 3; i++) swerveModules[i].setDesiredState( modules[i] ); }

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

    poseToTelemetry(actualPose, "Align/startPose");
    poseToTelemetry(closest, "Align/choosenWaypoint");
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
      commands.addCommands(
        _moveToPosition.generateMoveToPositionCommandTimed(
          waypoints.get( i ),
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
        _rotConstraints) );
    
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

  private void configDrive (TalonFX motor) {
    configDrive(motor, DRIVE_kP, DRIVE_kF);
  }

  // public to avoid warnings
  public void configAzimuth (TalonFX motor, CANCoder position) {
    configAzimuth(motor, position, AZIMUTH_kP, AZIMUTH_kD, AZIMUTH_kF);
  }

  private void configDrive (TalonFX motor, double kP, double kF) {
    motor.configFactoryDefault();
    motor.setInverted(TalonFXInvertType.CounterClockwise);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setSelectedSensorPosition(0);
    motor.config_kP(0, kP);
    motor.config_kF(0, kF);
    motor.configVoltageCompSaturation(12);
    motor.enableVoltageCompensation(true);
  }

  private void configAzimuth (TalonFX motor, CANCoder position, double kP, double kD, double kF) {
    motor.configFactoryDefault();
    motor.setInverted(TalonFXInvertType.CounterClockwise);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configRemoteFeedbackFilter(position, 0);
    motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    motor.configSelectedFeedbackCoefficient(360 / (2048 * AZIMUTH_GEAR_RATIO));
    motor.configStatorCurrentLimit(AZIMUTH_CURRENT_LIMIT);
    motor.setSelectedSensorPosition(degreesToFalcon(position.getAbsolutePosition()));
    motor.config_kP(0, kP);
    motor.config_kD(0, kD);
    motor.config_kF(0, kF);
    motor.configNeutralDeadband(AZIMUTH_DEADBAND);
  }

  private void configPosition (CANCoder encoder, double offset) {
    encoder.configFactoryDefault();
    encoder.configMagnetOffset(offset);
    encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    encoder.setPositionToAbsolute();
  }

  private void configPID(PIDController controller) {
    controller.enableContinuousInput(0, 360);
    controller.setTolerance(0);
  }

  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose = new Pose2d(
      speeds.vxMetersPerSecond * dt, 
      speeds.vyMetersPerSecond * dt, 
      new Rotation2d(speeds.omegaRadiansPerSecond * dt * 3)
    );
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  public void poseToTelemetry(Pose2d pose, String key) {
    Telemetry.setValue("drivetrain/" + key + "/x", pose.getTranslation().getX());
    Telemetry.setValue("drivetrain/" + key + "/y", pose.getTranslation().getY());
    Telemetry.setValue("drivetrain/" + key + "/heading", pose.getRotation().getDegrees());
  }

  public static double degreesToFalcon(double degrees) {
    return degrees / (360.0 / ( AZIMUTH_GEAR_RATIO * 2048.0));
  }
}