package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.PinchersofPower;
import static frc.robot.Constants.CAN.*;

public class RobotContainer {
  private final Pigeon m_gyro = new Pigeon(PIGEON_ID);
  private final Arm m_arm = new Arm();
  private final PinchersofPower m_claw;
  private final Drivetrain m_swerve = new Drivetrain(m_gyro);
  private final Limelight m_limelight = new Limelight();
  private final LEDs m_LEDs = new LEDs();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandGenericHID copilot = new CommandGenericHID(1);

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_claw = new PinchersofPower();

    driver = new CommandXboxController(0);
    copilot = new CommandGenericHID(1);

    m_gyro.zeroYaw();

    configureButtonBindings();
    m_swerve.setDefaultCommand(new DriveCommand(m_swerve, driver));
  }

  private void configureButtonBindings() {

    driver.a().onTrue(new InstantCommand(m_swerve::toggleRobotOrient, m_swerve));
    driver.b().onTrue(new InstantCommand(m_swerve::zeroGyro, m_swerve));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}