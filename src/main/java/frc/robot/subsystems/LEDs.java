package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.Telemetry;

public class LEDs extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private int m_r = 200;
  private int m_g = 233;
  private int m_b = 233;

  private int m_r2 = 0;
  private int m_g2 = 0;
  private int m_b2 = 0;

  public LEDs () {
    m_led = new AddressableLED(1);
   
    m_ledBuffer = new AddressableLEDBuffer(18);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setColor ( int r, int g, int b ) {
    System.out.println("\n \n TESTINGS \n \n.");
    m_r = r;
    m_g = g;
    m_b = b;
  }

  public Command turnPurple () { 
    return new InstantCommand( () -> 
      setColor(
        (int) Math.floor(Color.kPurple.red*255), 
        (int) Math.floor(Color.kPurple.green*255), 
        (int) Math.floor(Color.kPurple.blue*255)));
  }

  public Command turnRed () { 
    return new InstantCommand( () ->
      setColor( 
        (int) Math.floor(Color.kPurple.red*255), 
        (int) Math.floor(Color.kPurple.green*255), 
        (int) Math.floor(Color.kPurple.blue*255)));
   }

  public Command turnYellow () { 
    return new InstantCommand( () -> 
      setColor(
        (int) Math.floor(Color.kOrange.red*255), 
        (int) Math.floor(Color.kOrange.green*255), 
        (int) Math.floor(Color.kOrange.blue*255)));
  }

  public Command flashRed () { 
    return new InstantCommand( () -> {
      m_r2 = m_r;
      m_g2 = m_g;
      m_b2 = m_b;
      setColor(255, 0, 0);})
        .andThen( new WaitCommand(0.5) )
        .andThen( new InstantCommand( () -> setColor(m_r2, m_g2, m_b2)) ); 
    }

  public Command flashGreen () { 
    return new InstantCommand( () -> {
      m_r2 = m_r;
      m_g2 = m_g;
      m_b2 = m_b;
      setColor(0, 255, 0);})
        .andThen( new WaitCommand(0.5) )
        .andThen( new InstantCommand(() -> setColor(m_r2, m_g2, m_b2)) );
    }
  
  @Override
  public void periodic() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) m_ledBuffer.setRGB(i, m_r, m_g, m_b);

    m_led.setData(m_ledBuffer);
    m_led.start();

    Telemetry.setValue("LEDS/R", m_ledBuffer.getLED(9).red);
    Telemetry.setValue("LEDS/G", m_ledBuffer.getLED(9).green);
    Telemetry.setValue("LEDS/B", m_ledBuffer.getLED(9).blue);
  }

  @Override
  public void simulationPeriodic() {}
}