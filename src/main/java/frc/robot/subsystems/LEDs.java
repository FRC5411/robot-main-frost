package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Telemetry;
import frc.robot.Constants.*;

public class LEDs extends SubsystemBase {
  PWMSparkMax LEDS;
  String Mode;
  boolean success;
  int detect;

  public LEDs() {
    LEDS = new PWMSparkMax(0);
  }

  public void setMode(String State) {
    Mode = State;
  }

  public void setdetect(int bool) {
    detect = bool;
  }

  public void setsuccess(boolean bool) {
    success = bool;
  }

  public void detector() {
    if(detect == 1) {
      LEDS.set(LED.GREEN);
    }
    if(detect == -1) {
      LEDS.set(LED.RED);
    }
    detect = 0;
  }

  public void state() {
    if(Mode == "Cone") {
      LEDS.set(LED.YELLOW);
    }
    if(Mode == "Cube") {
      LEDS.set(LED.PURPLE);
    }
  }

  public void succeed() {
    if(success == true) {
      LEDS.set(LED.RAINBOW);
      Mode = "Neither";
    }
    LEDS.set(0);
  }

  @Override
  public void periodic() {
    detector();
    state();
    succeed();
    Telemetry.setValue("LEDS/motor/value", LEDS.get());
  }

  @Override
  public void simulationPeriodic() {}
}