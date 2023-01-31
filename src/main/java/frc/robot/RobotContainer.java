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
import frc.robot.subsystems.PinchersofPower;
import static frc.robot.Constants.CAN.*;

public class RobotContainer {
  private final Pigeon m_gyro = new Pigeon(PIGEON_ID);
  private final Arm m_arm = new Arm();
  private final PinchersofPower claw;
  private final Drivetrain m_swerve = new Drivetrain(m_gyro);
  private final Limelight m_limelight = new Limelight();
  private final LEDs m_LEDs = new LEDs();

  private final XboxController driver = new XboxController(0);
  private final GenericHID copilot = new GenericHID(1);
  
  Trigger aButton = new JoystickButton(driver, 1);
  Trigger bButton = new JoystickButton(driver, 2);

  public RobotContainer() {
    m_gyro.zeroYaw();
    claw = new PinchersofPower();

    configureButtonBindings();

    m_swerve.setDefaultCommand(new DriveCommand(m_swerve, driver));
  }

  private void configureButtonBindings() {
    aButton.onTrue(new InstantCommand(m_swerve::toggleRobotOrient, m_swerve));
    bButton.onTrue(new InstantCommand(m_swerve::zeroGyro, m_swerve));
  }

  public Command getAutonomousCommand(Command command) {
    return command;
  }
}