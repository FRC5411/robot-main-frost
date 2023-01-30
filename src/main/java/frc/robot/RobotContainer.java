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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Pigeon;
import static frc.robot.Constants.CAN.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Pigeon m_gyro = new Pigeon(PIGEON_ID);
  private final Arm m_arm = new Arm();
  private final Drivetrain m_swerve = new Drivetrain(m_gyro);
  private final Limelight m_limelight = new Limelight();
  private final LEDs m_LEDs = new LEDs();

  private final XboxController driver = new XboxController(0);
  private final GenericHID copilot = new GenericHID(1);
  
  Trigger aButton = new JoystickButton(driver, 1);
  Trigger bButton = new JoystickButton(driver, 2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_gyro.zeroYaw();

    // Configure the button bindings
    configureButtonBindings();

    m_swerve.setDefaultCommand(new DriveCommand(m_swerve, driver));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    aButton.onTrue(new InstantCommand(m_swerve::toggleRobotOrient, m_swerve));
    bButton.onTrue(new InstantCommand(m_swerve::zeroGyro, m_swerve));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
