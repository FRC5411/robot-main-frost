package frc.robot;
import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.ButtonBoard;
import frc.lib.Telemetry;

import frc.robot.Constants.ARM.positions;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.moveToObject;
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
  public Drivetrain m_swerve = new Drivetrain(m_gyro, m_arm, m_claw, vision);

  public moveToObject moveToObject = new moveToObject(vision, () -> m_claw.getPiece(), m_swerve, m_arm, m_claw);

  public RobotContainer() {
    File[] paths = new File(Filesystem.getDeployDirectory(), "pathplanner").listFiles();
    String pathsString = "";
    for (int i = 0; i < paths.length; i++) {
      if (paths[i].isDirectory()) continue;
      pathsString += paths[i].getName().substring(0, paths[i].getName().indexOf(".")) + ",";
    }
    Telemetry.setValue("general/autonomous/availableRoutines", pathsString);
    Telemetry.setValue("general/autonomous/selectedRoutine", "SET ME");

    SmartDashboard.putNumber("alignTranslateP", 5);//1.8;//3.25;//2.75;//2.5;//2.1;//2;//0.018;//0.03;//0.004 0.001
    SmartDashboard.putNumber("alignTranslateI", 0.1);
    SmartDashboard.putNumber("alignTranslateD", 0);
    SmartDashboard.putNumber("alignRotateP", 2);// 2v.5//12.5;//15;//0.00005
    SmartDashboard.putNumber("alignRotateI", 0);
    SmartDashboard.putNumber("alignRotateD", 0); // 0.1

    configureButtonBindings();

    m_arm.setDefaultCommand(m_arm.defaultCommand());
    m_swerve.setDefaultCommand(new DriveCommand(m_swerve, driverController, copilotController));
  }

  public Arm getArm() {
    return m_arm;
  }

  private void configureButtonBindings() {
    driverController.a().onTrue(new InstantCommand(m_swerve::zeroGyro));
    driverController.b().onTrue(new InstantCommand(m_swerve::toggleRobotOrient));
    driverController.y().onTrue(new InstantCommand(m_swerve::resetPoseWithLL));
      
    //driverController.x().whileTrue(new InstantCommand(() -> m_swerve.moveToPositionCommand().schedule()));
    driverController.x().whileTrue(new InstantCommand(() -> moveToObject.collectGamepiece().schedule()));
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
  }

  public void killRumble(){
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  public Command getAutonomousCommand() {
    return m_swerve.getAutonomousCommand().andThen(new InstantCommand( () -> m_swerve.stopModules()));
  }

  public static DriverStation.Alliance getDriverAlliance() {
    // What to do for competition
    //return DriverStation.getAlliance();

    // What to do for testing
    return DriverStation.getAlliance();
  }
}
