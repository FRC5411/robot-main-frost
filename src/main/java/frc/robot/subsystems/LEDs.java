package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class LEDs extends SubsystemBase {
  PWMSparkMax LEDS;
  String Mode;
  boolean detect;

  public LEDs() {
    LEDS = new PWMSparkMax(0);
  }

  public void setdetect(boolean bool) {
    detect = bool;
  }

  @Override
  public void periodic() {

    if(Mode == "Cone") {
      LEDS.set(LED.Yellow);
    }
    if(Mode == "Cone") {
      LEDS.set(LED.Yellow);
    }
    if(Mode == "Cone") {
      LEDS.set(LED.Yellow);
    }
    if(Mode == "Cube") {
      LEDS.set(LED.Purple);
    }
  }

  @Override
  public void simulationPeriodic() {}
}