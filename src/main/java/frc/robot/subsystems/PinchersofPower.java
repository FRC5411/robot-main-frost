package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Telemetry;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.POP;
import frc.robot.Constants.ARM.positions;

public class PinchersofPower extends SubsystemBase  {
  private final Compressor comp;
  private final DoubleSolenoid pusher;
  private RobotContainer m_container;
  private final CANSparkMax spinner;
  private final CANSparkMax spinner2;
  private final ColorSensorV3 colorSensor;
  private boolean m_cone;
  private double intakeSpeed = 0;
  private DigitalInput limitSwitch = new DigitalInput(Constants.DIO.GRIP_LIMIT_SWITCH);

  public PinchersofPower(RobotContainer m_container) {
    this.m_container = m_container;
    comp = new Compressor(Constants.CAN.PCH_ID, PneumaticsModuleType.REVPH);
    pusher = new DoubleSolenoid(Constants.CAN.PCH_ID, PneumaticsModuleType.REVPH, Constants.POP.FORWARD_PNEUMATIC_CHANNEL, Constants.POP.BACKWARD_PNEUMATIC_CHANNEL);
    spinner = new CANSparkMax(Constants.CAN.GRIP_LEFT_ID, MotorType.kBrushless);
    spinner2 = new CANSparkMax(Constants.CAN.GRIP_RIGHT_ID, MotorType.kBrushless);
    //spinner2.follow(spinner, true);
    colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    m_cone = true;

    //comp.disable();

    spinner.restoreFactoryDefaults();
    spinner2.restoreFactoryDefaults();

    spinner.clearFaults();
    spinner2.clearFaults();

    spinner.setIdleMode(IdleMode.kBrake);
    spinner2.setIdleMode(IdleMode.kBrake);

    spinner.setSmartCurrentLimit(20);
    spinner2.setSmartCurrentLimit(20);

    spinner2.setInverted(true);

    spinner.setCANTimeout(20);
    spinner2.setCANTimeout(20);

    spinner.burnFlash();
    spinner2.burnFlash();

  }

  /** Close */
  public void closeGrip() {
    pusher.set(Value.kForward);
  }

  /** Open */
  public void openGrip() {
    pusher.set(Value.kReverse);
  }

  public void toggle () {
    if (pusher.get() == Value.kReverse) {
      pusher.set(Value.kForward);
    } else {
      pusher.set(Value.kReverse);
    }
  }

  public void spinSlow() {
    intakeSpeed = POP.SPEEDIN/4;
  }

  public void spinIn() {
    intakeSpeed = POP.SPEEDIN;
  }

  public void spinOut() {
    intakeSpeed = -POP.SPEEDOUT;
  }

  public void spinOff() {
    intakeSpeed = 0;
  }

  public enum GamePieces {
    Cone,
    Cube,
    None
  }

  public GamePieces getColorSensorGamePiece () {
    Color actualColor = colorSensor.getColor();
    if ( colorSensor.getProximity() > 100 ) {
      if (actualColor.green < actualColor.blue) { // cube
        return GamePieces.Cube;
      } else if (actualColor.green < actualColor.blue) { // cone
        return GamePieces.Cone;
      } else {
        return GamePieces.None;
      }
    } else {
      return GamePieces.None;
    }
  }

  public void intake() {
    if(!m_cone) {
      openGrip();
    } else {
      closeGrip();
    }
  }
  public void setCone(boolean check){
    m_cone = check;
  }
  public boolean wantCone () {
    return m_cone;
  }

  public void setMode(GamePieces mode) {
    m_cone = (mode == GamePieces.Cone);
    spinSlow();

    if(!m_cone){
      openGrip();
    }
  }

  

  public Command intakeCommand() {
    return new InstantCommand(() -> intake(), this);
  }

  public Command outTakeCommand() {
    return new InstantCommand( () -> {
      if (m_container.getArm().target == positions.Substation && m_cone) {
        closeGrip();
      } else if ( m_cone ) {
        if ( m_container.getArm().target == positions.ScoreLow) {
          spinOut();  
        }
        spinOff();
        openGrip();
      } else {
        spinOut();
      }
    });
  }

  public Command spinOffCommand() {
    return new InstantCommand(() -> spinOff(), this);
  }

  @Override
  public void periodic() {
    if ( DriverStation.isEnabled() || DriverStation.isAutonomousEnabled() ) {
      spinner.set(intakeSpeed);
      spinner2.set(intakeSpeed);

      if ( !limitSwitch.get() && (m_container.m_arm.target == positions.Substation || m_container.m_arm.target == positions.Floor) && m_cone && !RobotContainer.copilotController.getRawButton(15) ) {
        closeGrip();
      }
    } else {
      // prevent CAN timeouts when disabled, actual motor stoppage is handled at a lower level
      spinner.set(0);
      spinner2.set(0);
    }

    Telemetry.setValue("Pincher/limitSwitch", !limitSwitch.get());
    Telemetry.setValue("Pincher/leftMotor/setpoint", spinner.get());
    Telemetry.setValue("Pincher/leftMotor/temperature", spinner.getMotorTemperature());
    Telemetry.setValue("Pincher/leftMotor/outputVoltage", spinner.getAppliedOutput());
    Telemetry.setValue("Pincher/leftMotor/statorcurrent", spinner.getOutputCurrent());
    Telemetry.setValue("Pincher/rightMotor/setpoint", spinner2.get());
    Telemetry.setValue("Pincher/rightMotor/temperature", spinner2.getMotorTemperature());
    Telemetry.setValue("Pincher/rightMotor/outputVoltage", spinner2.getAppliedOutput());
    Telemetry.setValue("Pincher/rightMotor/statorCurrent", spinner2.getOutputCurrent());
    Telemetry.setValue("Pincher/piston", pusher.get() == DoubleSolenoid.Value.kForward ? "Forward" : "Reverse");
    Telemetry.setValue("Pincher/compressor/pressure", comp.getPressure());
  }
}