package frc.robot;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.PinchersofPower;
import static frc.robot.Constants.CAN.*;

public class RobotContainer {
  private final Pigeon m_gyro = new Pigeon(PIGEON_ID);
  private final Arm m_arm = new Arm();
  private final PinchersofPower claw;
  private final Drivetrain m_swerve = new Drivetrain(m_gyro);
  private final Limelight m_limelight = new Limelight();
  private final LEDs m_LEDs = new LEDs();
  private final CommandXboxController driver;
  private final CommandGenericHID copilot;
  private final Trigger aButton;
  private final Trigger bButton;

  public RobotContainer() {
    claw = new PinchersofPower();

    driver = new CommandXboxController(0);
    copilot = new CommandGenericHID(1);

    aButton = driver.a();
    bButton = driver.b();
    
    m_gyro.zeroYaw();
    
    configureButtonBindings();

    m_swerve.setDefaultCommand(new DriveCommand(m_swerve, driver));
  }

  private void configureButtonBindings() {
    aButton.onTrue(new InstantCommand(m_swerve::toggleRobotOrient, m_swerve));
    bButton.onTrue(new InstantCommand(m_swerve::zeroGyro, m_swerve));
  }
}