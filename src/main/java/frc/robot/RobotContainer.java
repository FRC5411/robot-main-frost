package frc.robot;
import java.io.File;
import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.ButtonBoard;
import frc.lib.Telemetry;

import frc.robot.Constants.ARM.positions;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.PinchersofPower;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.PinchersofPower.GamePieces;

public class RobotContainer {
  // Im leaving these ports as magic constants because there's no case where they are not these values
  public static final CommandXboxController driverController = new CommandXboxController(0);
  public static final ButtonBoard copilotController = new ButtonBoard(1, 2);

  public Pigeon m_gyro = new Pigeon();
  public VisionSubsystem vision = new VisionSubsystem();
  public LEDs m_LEDs = new LEDs();
  public PinchersofPower m_claw = new PinchersofPower(this);
  public Arm m_arm = new Arm(m_claw, copilotController);
  public Drivetrain m_swerve = new Drivetrain();

  public RobotContainer() {
    File[] paths = new File(Filesystem.getDeployDirectory(), "pathplanner").listFiles();
    String pathsString = "";
    for (int i = 0; i < paths.length; i++) {
      if (paths[i].isDirectory()) continue;
      pathsString += paths[i].getName().substring(0, paths[i].getName().indexOf(".")) + ",";
    }
    Telemetry.setValue("general/autonomous/availableRoutines", pathsString);
    Telemetry.setValue("general/autonomous/selectedRoutine", "SET ME");

    configureButtonBindings();

    m_arm.setDefaultCommand(m_arm.defaultCommand());
    m_swerve.setDefaultCommand(new DriveCommand(m_swerve, driverController, copilotController));
  }

  public Arm getArm() {
    return m_arm;
  }

  private void configureButtonBindings() {
    driverController.a().onTrue(new InstantCommand(m_gyro::zeroYaw));
    driverController.b().onTrue(new InstantCommand(m_swerve::toggleRobotOrient));
    driverController.y().onTrue(new InstantCommand(m_swerve::resetPoseWithLL));
    // driverController.y().onTrue(
    //   new InstantCommand( () -> m_swerve.resetPose(new Pose2d(9 , 3, new Rotation2d())) ));
      
    driverController.x().whileTrue(new InstantCommand(() -> m_swerve.moveToPositionCommand(m_claw::wantCone).schedule()));
    driverController.x().onFalse(new InstantCommand(() -> {}, m_swerve));

    // driverController.y().onTrue(new InstantCommand(
    //   () -> m_swerve.resetPose(
    //     m_swerve.getTargetPose().plus(
    //       new Transform2d(
    //         new Translation2d(0.09, -0.09), 
    //         Rotation2d.fromDegrees(0.8))))));

    copilotController.button(0).whileTrue(m_arm.moveToPositionCommand(positions.Substation));
    copilotController.button(0).onFalse(m_claw.intakeCommand().alongWith(m_arm.moveToPositionCommand(positions.Idle)));

    copilotController.button(1).whileTrue(m_arm.moveToPositionCommand(positions.Floor));
    copilotController.button(1).onFalse(m_claw.intakeCommand());

    copilotController.button(2).onTrue(new InstantCommand( () -> m_arm.goToScoreHigh().schedule()));
    copilotController.button(2).onFalse(m_arm.defaultCommand());
    copilotController.button(2).onFalse(m_claw.intakeCommand());

    copilotController.button(3).whileTrue(m_arm.moveToPositionCommand(positions.FloorAlt));
    copilotController.button(3).onFalse(m_claw.intakeCommand());

    copilotController.button(4).whileTrue(new InstantCommand( () -> m_arm.goToScoreMid().schedule()));
    copilotController.button(4).onFalse(m_claw.intakeCommand());
    copilotController.button(4).onFalse(m_arm.defaultCommand());

    copilotController.button(5).whileTrue(m_arm.moveToPositionCommand(positions.ScoreLow));
    copilotController.button(5).onFalse(m_claw.intakeCommand());

    copilotController.button(6).onTrue(new SequentialCommandGroup((m_claw.outTakeCommand()), new WaitCommand(0.25), m_arm.moveToPositionCommand(positions.Idle)));
    copilotController.button(6).onFalse(m_claw.spinOffCommand());

    copilotController.button(7).onTrue(setGamePiece(GamePieces.Cube));
    copilotController.button(8).onTrue(setGamePiece(GamePieces.Cone));

    copilotController.button(9).onTrue(m_arm.defaultCommand().alongWith(m_arm.onManual()));
    copilotController.button(9).onFalse(m_arm.defaultCommand());
    
    copilotController.button(12).onTrue(new InstantCommand( () -> {
      if (copilotController.getRawButton(9)) {
        m_claw.toggle(); 
      }
    }));
    copilotController.button(14).whileTrue(new InstantCommand( () -> {
      if (copilotController.getRawButton(9)) {
        m_claw.spinOut();
      }
    }));
    copilotController.button(14).onFalse(new InstantCommand( () -> {if (copilotController.getRawButton(9)) m_claw.spinOff();}));
    copilotController.button(13).whileTrue(new InstantCommand( () -> {
      if (copilotController.getRawButton(9)) {
        m_claw.spinIn();
      }
    }));
    copilotController.button(13).onFalse(new InstantCommand( () -> {if (copilotController.getRawButton(9)) m_claw.spinOff();}));
  }

  public void killRumble(){
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  public Command getAutonomousCommand() {
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
    eventMap.put("autobalance", new AutoBalance(m_swerve, m_swerve.getGyro()));
    eventMap.put("coneMode", new InstantCommand( () -> { m_claw.setCone(true); m_claw.closeGrip(); m_claw.spinSlow(); } ));
    eventMap.put("cubeMode", new InstantCommand( () -> { m_claw.setCone(false); m_claw.openGrip(); } ));
    eventMap.put("wait", new WaitCommand(0.75));

    return m_swerve.getAutonomousCommand(eventMap, new SequentialCommandGroup(
      new InstantCommand( () -> m_swerve.stopModules() ),
      new InstantCommand( () -> { m_claw.setCone(true); m_claw.closeGrip(); } ),
      m_arm.moveToPositionTerminatingCommand(positions.ScoreHighCone).withTimeout(2.75).andThen(m_arm.moveToPositionCommand(positions.DipHighCone).withTimeout(0.75)),
      m_claw.outTakeCommand().andThen(new WaitCommand(.25)),
      m_arm.moveToPositionTerminatingCommand(positions.Idle) )).andThen(new InstantCommand( () -> m_swerve.stopModules()));
  }

  public static DriverStation.Alliance getDriverAlliance() {
    return DriverStation.getAlliance();
  }

  public static boolean isManual() {
    return copilotController.getRawButton(9);
  }

  public Command setGamePiece(GamePieces GP) {
    boolean isCone = GP.equals(GamePieces.Cone);
    Command updateMode = new InstantCommand( () -> {
      m_claw.setMode(GP);
      m_claw.setCone(isCone);
      copilotController.setLED(7, !isCone);
      copilotController.setLED(8, isCone);
    });

    if(isCone) return m_LEDs.turnYellow().alongWith(updateMode);
    else return m_LEDs.turnPurple().alongWith(updateMode);
  }

  public static void setCopilotLEDs() {
    if (!DriverStation.isAutonomous()) {
      setManualLEDs(isManual());
      setAutoLEDs(!isManual());
    }
    else {
      setManualLEDs(false);
      setAutoLEDs(false);
    }
  }

  public static void setManualLEDs(boolean isOn) {
    for(int i = 0; i <= 6; i++ ) copilotController.setLED(i, isOn);
  }

  public static void setAutoLEDs(boolean isOn) {
    for(int i = 10; i <= 14; i++) copilotController.setLED(i, isOn);
  }
}
