package frc.robot.subsystems;
import static frc.robot.Constants.DRIVETRAIN.*;
import static frc.robot.Constants.CAN.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Pigeon;
import frc.lib.SimpleUtils;
import frc.lib.SwerveModule;
import frc.lib.Telemetry;

import frc.robot.commands.moveToPosition;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax shwerveDrive;
  private final SwerveModule[] swerveModules;
  private final VisionSubsystem vision;
  private final Pigeon gyro;

  private ChassisSpeeds m_chassisSpeeds;
  private ChassisSpeeds forwardKinematics;
  private SwerveModuleState[] moduleStates;
  private SwerveModulePosition[] modulePoses;
  private Pose2d _robotPose = new Pose2d();
  private Pose2d _lastPose = _robotPose;
  private SwerveDrivePoseEstimator odometry;

  private boolean isRobotOriented = false;

  private List<Pose2d> _coneWaypoints;
  private List<Pose2d> _cubeWaypoints;

  private moveToPosition _moveToPosition;

  public Drivetrain() {
    shwerveDrive = new CANSparkMax(SHWERVE_DRIVE_ID, MotorType.kBrushless);

    swerveModules = new SwerveModule[] {
      new SwerveModule( 
        FL_DRIVE_ID, FL_AZIMUTH_ID, FL_CANCODER_ID, 
        FL_PID, FL_ECODER_OFFSET, FL_kF, "FL" ),
      new SwerveModule( 
        FR_DRIVE_ID, FR_AZIMUTH_ID, FR_CANCODER_ID, 
        FR_PID, FR_ECODER_OFFSET, FR_kF, "FR" ),
      new SwerveModule( 
        BL_DRIVE_ID, BL_AZIMUTH_ID, BL_CANCODER_ID, 
        BL_PID, BL_ECODER_OFFSET, BL_kF, "BL" ),
      new SwerveModule( 
        BR_DRIVE_ID, BR_AZIMUTH_ID, BR_CANCODER_ID, 
        BR_PID, BR_ECODER_OFFSET, BR_kF, "BR" ),
    };

    vision = new VisionSubsystem();
    gyro = new Pigeon();

    m_chassisSpeeds = new ChassisSpeeds();
    forwardKinematics = new ChassisSpeeds();
    moduleStates = new SwerveModuleState[4];
    modulePoses  = new SwerveModulePosition[4];
    odometry = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), getSwerveModulePositions(), _robotPose);

    _coneWaypoints = new ArrayList<Pose2d>();
    _cubeWaypoints = new ArrayList<Pose2d>();
    if      (RobotContainer.getDriverAlliance().equals(DriverStation.Alliance.Red ))  loadRedWayPoints();
    else if (RobotContainer.getDriverAlliance().equals(DriverStation.Alliance.Blue)) loadBlueWayPoints();

    _moveToPosition = new moveToPosition(
      this::getPose,
      this::getChassisSpeeds,
      this::driveFromChassisSpeeds,
      this );

    PathPlannerServer.startServer(6969);

    if(vision.getCenterLimelight().hasTarget()) resetPoseWithLL();;
  }

  @Override
  public void periodic() {
    Telemetry.setValue("drivetrain/isRobotOriented", isRobotOriented);
    for(int i = 0; i < swerveModules.length; i++) swerveModules[i].telemetry();
    gyro.periodic();

    _robotPose = odometry.update(gyro.getRotation2d(), getSwerveModulePositions());
    vision.addVisionMeasurement( odometry );
    forwardKinematics = SimpleUtils.transformToChassisSpeeds( _robotPose.minus( _lastPose ).div( 0.02 ) );

    SimpleUtils.poseToTelemetry( _robotPose, "/chassis/" );
    SimpleUtils.chassisToTelmetry(m_chassisSpeeds, "/chassis/" );

    _lastPose = _robotPose;

    field2d.setRobotPose(_robotPose);
    SmartDashboard.putData(field2d);
  }

  ///////////////////////////////////// DRIVE FUNCTIONS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
  public void joystickDrive( double LX, double LY, double RX ) {
    m_chassisSpeeds = new ChassisSpeeds(
      LY * MAX_LINEAR_SPEED, 
     -LX * MAX_LINEAR_SPEED, 
     -RX * MAX_ROTATION_SPEED );
    if ( !isRobotOriented ) 
      m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds( m_chassisSpeeds, gyro.getAllianceRotation2d() );
    setDesiredStates();
  }

  // For autonomous
  public void driveFromModuleStates ( SwerveModuleState[] modules ) {
    m_chassisSpeeds = kinematics.toChassisSpeeds( modules );
    setDesiredStates();
  }

  public void driveFromChassisSpeeds ( ChassisSpeeds speeds ) {
    m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds( speeds, gyro.getRotation2d() );
    setDesiredStates();
  }

  public void setDesiredStates() { 
    m_chassisSpeeds = SimpleUtils.discretize( m_chassisSpeeds );
    moduleStates = kinematics.toSwerveModuleStates( m_chassisSpeeds );
    SwerveDriveKinematics.desaturateWheelSpeeds( moduleStates, MAX_LINEAR_SPEED );
    for(int i = 0; i <= 3; i++) swerveModules[i].setDesiredState( moduleStates[i] );
  }

  public void shwerve ( double LX, double LY ) {
    if (isRobotOriented) shwerveDrive.set( MathUtil.clamp( -LX * 9, -1, 1 ) );
    else noShwerve();
  }

  public void stopModules () { for(int i = 0; i <= 3; i++) swerveModules[i].stopMotors(); }

  public void noShwerve () { shwerveDrive.stopMotor(); }

  //////////////////////////////////////// AUTO-ALIGNMENT \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
  public Command moveToPositionCommand (BooleanSupplier coneOrCube) {
    Pose2d actualPose = _robotPose; 

    Pose2d closest = actualPose.nearest( coneOrCube.getAsBoolean() ? _coneWaypoints : _cubeWaypoints );
    if (closest == null) return new InstantCommand();

    if(_robotPose.getX() >= 14.05 || _robotPose.getX() < 8) return pathToCommand( closest );
    return pathToCommand( _moveToPosition.optimizeWaypoints( closest ) );
  }

  public Command pathToCommand ( Pose2d target ) {
    Pose2d edgePose = new Pose2d(
      _robotPose.getX(), 
      target.getY(), 
      target.getRotation() );

    field2d.getObject("edgeTarget").setPose(edgePose);
    field2d.getObject("target").setPose(target);

    return new SequentialCommandGroup(
      _moveToPosition.generateMoveToPositionCommandTimed(
        edgePose,
        defaultTolerance,
        _holonomicConstraints,
        generateAlignmentController() ),
      _moveToPosition.generateMoveToPositionCommand( 
        target,
        new Pose2d(), 
        generateAlignmentController() ) );
  }

  public Command pathToCommand( List<Pose2d> waypoints ) {
    field2d.getObject("Targets").setPoses(waypoints);
    SequentialCommandGroup commands = new SequentialCommandGroup();

    for(int i = 0; i < waypoints.size() - 1; i++) {
      commands.addCommands(
        _moveToPosition.generateMoveToPositionCommandTimed(
          waypoints.get( i ),
          defaultTolerance,
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

  private void loadRedWayPoints() {
    _coneWaypoints.add(new Pose2d(0.76, 6.13, Rotation2d.fromDegrees(180)));
    _coneWaypoints.add(new Pose2d(0.76, 7.49, Rotation2d.fromDegrees(180)));
    _coneWaypoints.add(new Pose2d(14.75, 4.98, new Rotation2d()));
    _coneWaypoints.add(new Pose2d(14.75, 3.94 - 0.05, new Rotation2d()));
    _coneWaypoints.add(new Pose2d(14.75, 3.38 - 0.05, new Rotation2d()));
    _coneWaypoints.add(new Pose2d(14.75, 2.28 - 0.05, new Rotation2d()));
    _coneWaypoints.add(new Pose2d(14.75, 1.67, new Rotation2d()));
    _coneWaypoints.add(new Pose2d(14.75, 0.47 + 0.05, new Rotation2d()));

    _cubeWaypoints.add(new Pose2d(0.76, 6.13, Rotation2d.fromDegrees(180)));
    _cubeWaypoints.add(new Pose2d(0.76, 7.49, Rotation2d.fromDegrees(180)));
    _cubeWaypoints.add(new Pose2d(14.75, 1.13 - 0.05, new Rotation2d()));
    _cubeWaypoints.add(new Pose2d(14.75, 2.95 - 0.05, new Rotation2d()));
    _cubeWaypoints.add(new Pose2d(14.75, 4.52 - 0.05, new Rotation2d()));
  }

  private void loadBlueWayPoints() {
    _coneWaypoints.add(new Pose2d(0.76, 6.13, Rotation2d.fromDegrees(180)));
    _coneWaypoints.add(new Pose2d(0.76, 7.49, Rotation2d.fromDegrees(180)));
    _coneWaypoints.add(new Pose2d(14.75, 4.98, new Rotation2d()));
    _coneWaypoints.add(new Pose2d(14.75, 3.94 - 0.05, new Rotation2d()));
    _coneWaypoints.add(new Pose2d(14.75, 3.38 - 0.05, new Rotation2d()));
    _coneWaypoints.add(new Pose2d(14.75, 2.28 - 0.05, new Rotation2d()));
    _coneWaypoints.add(new Pose2d(14.75, 1.67, new Rotation2d()));
    _coneWaypoints.add(new Pose2d(14.75, 0.47 + 0.05, new Rotation2d()));

    _cubeWaypoints.add(new Pose2d(0.76, 6.13, Rotation2d.fromDegrees(180)));
    _cubeWaypoints.add(new Pose2d(0.76, 7.49, Rotation2d.fromDegrees(180)));
    _cubeWaypoints.add(new Pose2d(14.75, 1.13 - 0.05, new Rotation2d()));
    _cubeWaypoints.add(new Pose2d(14.75, 2.95 - 0.05, new Rotation2d()));
    _cubeWaypoints.add(new Pose2d(14.75, 4.52 - 0.05, new Rotation2d()));
  }

  /////////////////////////////// AUTONOMOUS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
  public Command getAutonomousCommand (HashMap<String, Command> eventMap, Command backUp) {
      if (Telemetry.getValue("general/autonomous/selectedRoutine", "dontMove").equals("special")) {
        return new SequentialCommandGroup(
          new InstantCommand( ()-> setRobotOriented(true) ),
          new InstantCommand( ()-> joystickDrive(0, 0.5, 0) ).repeatedly().withTimeout(1),
          new InstantCommand( () -> stopModules() ));
      }
      try {
        return new SwerveAutoBuilder( this::getPose, this::resetPose, kinematics,
          _translationPID, _rotationPID, this::driveFromModuleStates, eventMap,
          true, this).fullAuto( PathPlanner.loadPathGroup(
            Telemetry.getValue("general/autonomous/selectedRoutine", "dontMove"),
            PathPlanner.getConstraintsFromPath(
              Telemetry.getValue("general/autonomous/selectedRoutine", "Mobility") ) ) );
      } catch (Exception e) {
        DriverStation.reportError("it crashed LOL " + e.getLocalizedMessage(), true);
        return backUp;
      }
    }

  //////////////////////////////////////////// GETTERS AND SETTERS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

  public boolean getIsRobotOriented() { 
    return isRobotOriented; 
  }
  
  public Pigeon getGyro() {
    return gyro;
  }

  public VisionSubsystem getVision() {
    return vision;
  }

  public boolean toggleRobotOrient() { 
    return isRobotOriented = !isRobotOriented; 
  }

  public void setRobotOriented(boolean _isRobotOriented) { 
    isRobotOriented = _isRobotOriented; 
  }

  public void resetPoseWithLL() { 
    resetPose(
      new Pose2d(
        vision.getCenterLimelight().getPose().getX(), 
        vision.getCenterLimelight().getPose().getY(), 
        gyro.getRotation2d() ) );
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
      gyro.getRotation2d(), 
      getSwerveModulePositions(), 
      pose);
  }

  public ChassisSpeeds getChassisSpeeds() { 
    return forwardKinematics; 
  }

  public SwerveDriveKinematics getKinematics() { 
    return kinematics; 
  }

  public Pose2d getTargetPose() { 
    return _moveToPosition.getTarget(); 
  }

  private SwerveModulePosition[] getSwerveModulePositions() {
    for(int i = 0; i <= 3; i++) modulePoses[i] = swerveModules[i].getPosition();
    return modulePoses;
  }

  public Pose2d getPose() { 
    return _robotPose; 
  }
}