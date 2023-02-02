package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Telemetry;
import frc.robot.Constants.POP;

public class PinchersofPower extends SubsystemBase 
{
  private final Compressor comp;
  private final DoubleSolenoid pusher;
  private final CANSparkMax spinner;
  private boolean m_cone;

  public PinchersofPower() {
    comp = new Compressor(1, PneumaticsModuleType.CTREPCM);
    pusher = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    spinner = new CANSparkMax(25, MotorType.kBrushless);
    m_cone = false;
  }

  //Close Gripper
  public void forward() {
    pusher.set(Value.kForward);
  }

  //Open Gripper
  public void reverse() {
    pusher.set(Value.kReverse);
  }

  public void off() {
    pusher.set(Value.kOff);
  }

  //Spin Gripper Inwards
  public void spinin() {
    spinner.set(POP.SPEED);
  }

  //Spin Gripper Outwards
  public void spinout() {
    spinner.set(-POP.SPEED);
  }

  public void spinoff() {
    spinner.set(0);
  }

  public void enable() {
    comp.enableDigital();
  }

  public void disable() {
    comp.disable();
  }

  public void intake() {
    if((pusher.get() == Value.kForward)) {
      reverse();
    }
    spinin();
    if((pusher.get() != Value.kForward) && (m_cone == true)) {
      forward();
    }
  }

  public void outtake() {
    if(m_cone != true) {
      spinout();
    }
    if((m_cone == true) && (pusher.get() == Value.kForward)) {
      pusher.set(Value.kReverse);
    }
  }

  public void notake() {
    spinoff();
    off();
  }

  public void setMode(String mode) {
    if(mode == "cone") {
      m_cone = true;
    }
    if(mode == "cube") {
      m_cone = false;
    }
  }

  public Command Intake(PinchersofPower Claw) {
    return new InstantCommand(() -> intake(), Claw);
  }

  public Command Outtake(PinchersofPower Claw) {
    return new InstantCommand(() -> outtake(), Claw);
  }

  public Command Notake(PinchersofPower Claw) {
    return new InstantCommand(() -> notake(), Claw);
  }

  @Override
  public void periodic() {
    Telemetry.setValue("POP/motor/speed", spinner.get());
    Telemetry.setValue("POP/motor/temp", spinner.getMotorTemperature());
    Telemetry.setValue("POP/motor/voltage", spinner.getAppliedOutput());
    Telemetry.setValue("POP/motor/statorcurrent", spinner.getOutputCurrent());
    Telemetry.setValue("POP/pneumatics/value", pusher.get());
  }
}