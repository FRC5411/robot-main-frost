package frc.robot.subsystems;
import static frc.robot.Constants.DRIVETRAIN.*;
import static frc.robot.Constants.CAN.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.SimpleUtils;
import frc.lib.SwerveModule;
import frc.lib.Telemetry;

import frc.robot.commands.HolonomicController;
import frc.robot.commands.moveToPosition;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds forwardKinematics = new ChassisSpeeds();

  private SwerveModuleState[] moduleStates = new SwerveModuleState[4];
  private SwerveModulePosition[] modulePoses = new SwerveModulePosition[4];

  private final CANSparkMax shwerveDrive = new CANSparkMax(SHWERVE_DRIVE_ID, MotorType.kBrushless);

  private final SwerveModule[] swerveModules = new SwerveModule[] {
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

  private final VisionSubsystem vision = new VisionSubsystem();
  private final Pigeon gyro = new Pigeon();

  private boolean isRobotOriented = false;

  private SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
    kinematics,
    new Rotation2d(), 
    getSwerveModulePositions(), 
    new Pose2d());;

  private List<Pose2d> _coneWaypoints = new ArrayList<Pose2d>();
  private List<Pose2d> _cubeWaypoints = new ArrayList<Pose2d>();

  private Pose2d _robotPose = new Pose2d();
  private Pose2d _lastPose = _robotPose;

  private moveToPosition _moveToPosition = new moveToPosition(
    this::getPose,
    this::getChassisSpeeds,
    this::driveFromChassisSpeeds,
    this );

  public Drivetrain() {
    if      (RobotContainer.getDriverAlliance().equals(DriverStation.Alliance.Red ))  loadRedWayPoints();
    else if (RobotContainer.getDriverAlliance().equals(DriverStation.Alliance.Blue)) loadBlueWayPoints();

    PathPlannerServer.startServer(6969);

    if(vision.getCenterLimelight().hasTarget()) resetPoseWithLL();;
  }

  @Override
  public void periodic() {
    //////////// MODULES \\\\\\\\\\\\
    for(int i = 0; i < swerveModules.length; i++) swerveModules[i].telemetry();
    /////////////////////////

    ///////////// Pose \\\\\\\\\\\\\\\\\
    Telemetry.setValue("drivetrain/isRobotOriented", isRobotOriented);
    // Telemetry.setValue("e", modules);
    if ( vision.getCenterLimelight().hasTarget() ) 
      odometry.addVisionMeasurement(
        vision.getCenterLimelight().getPose(), 
        Timer.getFPGATimestamp() - vision.getCenterLimelight().getLatency(),
        VecBuilder.fill(
          3.1 * vision.getCenterLimelight().getTarget().getTranslation().getNorm(), 
          3.1 * vision.getCenterLimelight().getTarget().getTranslation().getNorm(), 10000000) );

    _robotPose = odometry.update(new Rotation2d(Math.toRadians(gyro.getYaw())), getSwerveModulePositions());

    Transform2d transform = _robotPose.minus(_lastPose).div(0.02);

    forwardKinematics = new ChassisSpeeds(
      transform.getX(), 
      transform.getY(), 
      transform.getRotation().getDegrees() );

    SimpleUtils.poseToTelemetry( _robotPose, "/chassis/" );
    Telemetry.setValue("drivetrain/chassis/robot/forwardSpeed", forwardKinematics.vxMetersPerSecond);
    Telemetry.setValue("drivetrain/chassis/robot/rightwardSpeed", forwardKinematics.vyMetersPerSecond);
    Telemetry.setValue("drivetrain/chassis/clockwiseSpeed", Math.toDegrees(forwardKinematics.omegaRadiansPerSecond));

    _lastPose = _robotPose;

    field2d.setRobotPose(_robotPose);
    SmartDashboard.putData(field2d);   
    /////////////////////////////////////

    // double[] e = new double[8];
    // e[0] = modules[0].angle.getRadians();
    // e[1] = modules[0].speedMetersPerSecond;
    // e[2] = modules[1].angle.getRadians();
    // e[3] = modules[2].speedMetersPerSecond;
    // SmartDashboard.putNumberArray("e", e);
  }

  ///////////////////////////////////// DRIVE FUNCTIONS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
  public void joystickDrive(double LX, double LY, double RX) {
    if ( !isRobotOriented ) m_chassisSpeeds = 
      ChassisSpeeds.fromFieldRelativeSpeeds(
        LY * MAX_LINEAR_SPEED,
        -LX * MAX_LINEAR_SPEED,
        -RX * MAX_ROTATION_SPEED, 
        odometry.getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(
          (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) ? 180 : 0 ) ) );

    else m_chassisSpeeds = new ChassisSpeeds(LY * MAX_LINEAR_SPEED, -LX * MAX_LINEAR_SPEED, -RX * MAX_ROTATION_SPEED);

    m_chassisSpeeds = SimpleUtils.discretize( m_chassisSpeeds );
    moduleStates = kinematics.toSwerveModuleStates( m_chassisSpeeds );

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_LINEAR_SPEED);
    setDesiredStates();
  }

  // For autonomous
  public void driveFromModuleStates ( SwerveModuleState[] modules ) {
    modules = kinematics.toSwerveModuleStates( SimpleUtils.discretize( kinematics.toChassisSpeeds(modules) ) );
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);
    setDesiredStates();
  }

  public void driveFromChassisSpeeds (ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds, odometry.getEstimatedPosition().getRotation() );

    moduleStates = kinematics.toSwerveModuleStates( speeds );
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_LINEAR_SPEED);
    setDesiredStates();
  }

  public void stopModules () { 
    for(int i = 0; i <= 3; i++) swerveModules[i].stopMotors(); 
  }

  public void setDesiredStates() { 
    for(int i = 0; i <= 3; i++) swerveModules[i].setDesiredState( moduleStates[i] ); 
  }

  public void shwerve ( double LX, double LY) {
    // 6in diameter wheels, 10:1 gearbox
    if (isRobotOriented) shwerveDrive.set(MathUtil.clamp(-LX*9, -1, 1));
    else noShwerve();
  }

  public void noShwerve () {
    shwerveDrive.set(0);
  }

  //////////////////////////////////////// AUTO-ALIGNMENT \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
  public Command moveToPositionCommand (BooleanSupplier coneOrCube) {
    Pose2d actualPose = _robotPose; 

    Pose2d closest = actualPose.nearest( coneOrCube.getAsBoolean() ? _coneWaypoints : _cubeWaypoints );
    if (closest == null) return new InstantCommand();

    SimpleUtils.poseToTelemetry(actualPose, "Align/startPose");
    SimpleUtils.poseToTelemetry(closest, "Align/choosenWaypoint");
    if(_robotPose.getX() >= 14.05 || _robotPose.getX() < 8) return pathToCommand( closest );
    return pathToCommand( _moveToPosition.optimizeWaypoints( closest ) );
  }

  public Command pathToCommand (Pose2d target) {
    Pose2d edgePose = new Pose2d(
      _robotPose.getX(), 
      target.getY(), 
      target.getRotation() );

    field2d.getObject("targetEdge").setPose(edgePose);
    field2d.getObject("target").setPose(target);

    Command toAlign = _moveToPosition.generateMoveToPositionCommandTimed(
      edgePose,
      new ChassisSpeeds(0.75, 0.0, 0.0),
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

    for(int i = 0; i < waypoints.size() - 1; i++)
      commands.addCommands(
        _moveToPosition.generateMoveToPositionCommandTimed(
          waypoints.get(i),
          new Pose2d( 0.1, 0.1, Rotation2d.fromDegrees(3) ),
          _holonomicConstraints,
          generateAlignmentController() ) );

    commands.addCommands(
      _moveToPosition.generateMoveToPositionCommand(
        waypoints.get(waypoints.size() - 1 ),
        new Pose2d(), 
        generateAlignmentController() ));

    return commands;
  }

  public HolonomicController generateAlignmentController() {
    HolonomicController controller = new HolonomicController(
      new ProfiledPIDController(
        _alignXKp, 
        _alignXKi,
        _alignXKd,
        _XConstraints), 
      new ProfiledPIDController(
        _alignYKp, 
        _alignYKi,
        _alignYKd,
        _YConstraints), 
      new ProfiledPIDController(
        _alignRotationKp, 
        _alignRotationKi,
        _alignRotationKd,
        _rotConstraints) );
    
    controller.xControllerIRange(-0.75, 0.75);
    controller.yControllerIRange(-0.5, 0.5);
    controller.thetaControllerIRange(-8.5, 8.5);

    return controller;
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
        return new InstantCommand(()->setRobotOriented(true)).andThen(new RepeatCommand(new InstantCommand(()->joystickDrive(0, 0.5, 0))).withTimeout(1).andThen(new InstantCommand(()->stopModules())));
      }
  
      // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
      // for every path in the group
      try {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
          Telemetry.getValue("general/autonomous/selectedRoutine", "dontMove"),
          PathPlanner.getConstraintsFromPath(
            Telemetry.getValue("general/autonomous/selectedRoutine", "Mobility")));
  
        return new SwerveAutoBuilder(
          this::getPose,
          this::resetPose,
          kinematics,
          new PIDConstants(_translationKp, _translationKi, _translationKd),
          new PIDConstants(_rotationKp, _rotationKi, _rotationKd),
          this::driveFromModuleStates,
          eventMap,
          true,
          this
          ).fullAuto( pathGroup );

      } catch (Exception e) {
        // uh oh
        DriverStation.reportError("it crashed LOL " + e.getLocalizedMessage(), true);
        // score a preloaded cone if the auton crashes
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
        Rotation2d.fromDegrees( gyro.getYaw() ) ) );
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
      Rotation2d.fromDegrees( gyro.getYaw() ), 
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