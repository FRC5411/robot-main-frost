package frc.robot;
import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.PinchersofPower;
import frc.robot.subsystems.PinchersofPower.GamePieces;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Im leaving these ports as magic constants because there's no case where they are not these values
  public static final CommandXboxController driverController = new CommandXboxController(0);
  public static final ButtonBoard copilotController = new ButtonBoard(1, 2);

  // The robot's subsystems and commands are defined here...
  public Pigeon m_gyro = new Pigeon();
  public Limelight m_limelight = new Limelight();
  public LEDs m_LEDs = new LEDs();
  public PinchersofPower m_claw = new PinchersofPower(this);
  public Arm m_arm = new Arm(m_claw, copilotController);
  public Drivetrain m_swerve = new Drivetrain(driverController, m_gyro, m_arm, m_claw, m_limelight, m_LEDs);
  //private UsbCamera rawCamera;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Telemetry.flushTable();

    File[] paths = new File(Filesystem.getDeployDirectory(), "pathplanner").listFiles();
    String pathsString = "";
    for (int i = 0; i < paths.length; i++) {
      if (paths[i].isDirectory()) continue;
      pathsString += paths[i].getName().substring(0, paths[i].getName().indexOf(".")) + ",";
    }
    Telemetry.setValue("general/autonomous/availableRoutines", pathsString);
    Telemetry.setValue("general/autonomous/selectedRoutine", "SET ME");

    // Configure the button bindings
    configureButtonBindings();

   // m_LEDs.rainbow();
    m_arm.setDefaultCommand(m_arm.defaultCommand());
    m_swerve.setDefaultCommand(new DriveCommand(m_swerve, driverController, copilotController));

    //rawCamera = CameraServer.startAutomaticCapture("Aim Camera", 0);
    //rawCamera.setFPS(30);
    //rawCamera.setResolution(640, 480);
  }

  public Arm getArm() {
    return m_arm;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverController.x()
      .whileTrue(m_swerve.PPpathToCommand(m_limelight.getPose()))
      .onFalse(new InstantCommand(() -> m_swerve.stopModules()));
      
    driverController.a().onTrue(new InstantCommand(m_swerve::zeroGyro));
    driverController.b().onTrue(new InstantCommand(m_swerve::toggleRobotOrient));
    driverController.y().whileTrue(new AutoBalance(m_swerve));
    // This command uses the robot odometry to drive to a specific location on the field
    driverController.x().onTrue(m_swerve.PPmoveToPositionCommand());
    copilotController.button(0).whileTrue(m_arm.moveToPositionCommand(positions.Substation));
    copilotController.button(0).onFalse(m_claw.intakeCommand().alongWith(m_arm.moveToPositionCommand(positions.Idle)));
    copilotController.button(1).whileTrue(m_arm.moveToPositionCommand(positions.Floor));
    copilotController.button(1).onFalse(m_claw.intakeCommand());
    copilotController.button(2).onTrue(new InstantCommand( () -> m_arm.goToScoreHigh().schedule()));
//    copilotController.button(2).onFalse(new SequentialCommandGroup((m_arm.moveToPositionCommand(positions.Idle)).withTimeout(10), new WaitCommand(3.5), m_claw.intakeCommand())); //Close claw after the arm moves away from the node
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
    copilotController.button(8).onTrue(m_LEDs.turnYellow().alongWith(new InstantCommand( () -> m_claw.setMode(GamePieces.Cone))).alongWith(new InstantCommand( () -> m_claw.setCone(true)).alongWith(new InstantCommand( () -> {copilotController.setLED(7, false);copilotController.setLED(8, true);}))));
    copilotController.button(7).onTrue(m_LEDs.turnPurple().alongWith(new InstantCommand( () -> m_claw.setMode(GamePieces.Cube))).alongWith(new InstantCommand( () -> m_claw.setCone(false)).alongWith(new InstantCommand( () -> {copilotController.setLED(7, true);copilotController.setLED(8, false);}))));
    copilotController.button(8).onTrue(new InstantCommand( () -> m_claw.setCone(true)));
    copilotController.button(7).onTrue(new InstantCommand( () -> m_claw.setCone(false)));
    copilotController.button(6).onFalse(m_claw.spinOffCommand());
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
    
    //driverController.axisGreaterThan(2, 0.1).onTrue(m_swerve.moveToPositionCommand());
    //driverController.axisGreaterThan(3, 0.1).onTrue(m_swerve.moveToPositionCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // InstantCommand to set speeds to 0
    return m_swerve.getAutonomousCommand().andThen(new InstantCommand( () -> m_swerve.stopModules()));
  }

  public static DriverStation.Alliance getDriverAlliance() {
    // What to do for competition
    //return DriverStation.getAlliance();

    // What to do for testing
    return DriverStation.Alliance.Red;
  }
}