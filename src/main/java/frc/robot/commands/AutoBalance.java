package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pigeon;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends ProfiledPIDCommand {
  /** Creates a new AutoBalance. */
  private Drivetrain drivetrain;
  public AutoBalance(Drivetrain drivetrain, Pigeon gyro) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.005,
            0,
            0.00006,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1.5, 1)),
        // This should return the measurement
        gyro::getPitch, //gyro X angle
        // This should return the goal (can also be a constant)
        0,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.joystickDrive(-output, 0, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    getController().setTolerance(33.0); //degrees

  }
  
  public void initialize() {
    drivetrain.setRobotOriented(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void end() {
    drivetrain.setRobotOriented(false);
    drivetrain.joystickDrive(0,0,0);
  }
}