//Root Package
package frc.robot.subsystems;

//Libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.POP;

//TODO: Gripping Subsystem

/**
 * Gripping Subsystem Class
 */
public class PinchersofPower extends SubsystemBase 
{
  //Instance Variables
  private final DoubleSolenoid pusher;
  private final CANSparkMax spinner;

  /**
   * Gripping Class Constructor
   */
  public PinchersofPower() 
  {
    pusher = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    spinner = new CANSparkMax(25, MotorType.kBrushless);
  }

  //Close Gripper
  public void push() 
  {
    pusher.set(Value.kForward);
  }

  //Open Gripper
  public void reverse() {
    pusher.set(Value.kReverse);
  }

  //Spin Gripper Inwards
  public void spinin() 
  {
    spinner.set(POP.SPEED);
  }

  //Spin Gripper Outwards
  public void spinout() 
  {
    spinner.set(-POP.SPEED);
  }

  @Override
  public void periodic() {}
}
