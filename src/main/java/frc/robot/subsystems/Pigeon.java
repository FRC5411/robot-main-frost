//Root Package
package frc.robot.subsystems;

//Libraries
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Telemetry;

/**
 * Pigeon Gyroscope Wrapper Class
 */
public class Pigeon extends SubsystemBase 
{
  //Primary Gyroscope
  private Pigeon2 M_Gyro;

  //Gyroscope Data
  private double Yaw = 0;
  private double Pitch = 0;
  private double Roll = 0;
  private double[] YPR;

  private double yaw = 0;
  private double pitch = 0;
  private double roll = 0;
  private double[] ypr = new double[3];

  /**
   * Return the M_Gyro
   */
  public Pigeon2 getGyro () {return M_Gyro;}


  /**
   * Return the M_Gyro Yaw as double
   */
  public double getYaw() {return Yaw;}

  /**
   * Return the M_Gyro Pitch as double
   */
  public double getPitch () {return Pitch;}

  /**
   * Return the M_Gyro Roll as double
   */
  public double getRoll () {return Roll;}

  /**
   * Return the M_Gyro Yaw,Pitch,Roll as double list
   */
  public double[] getYPR () {return YPR;}

  /**
   * Zero the M_Gyro Yaw at current value
   */
  public void zeroYaw () {M_Gyro.setYaw(0);}

  /**
   * Pigeon Class Periodic
   */
  @Override
  public void periodic() 
  {
    //Update Data
    Yaw = M_Gyro.getYaw();
    Pitch = M_Gyro.getPitch();
    Roll = M_Gyro.getRoll();
    M_Gyro.getYawPitchRoll(YPR);

    //Telemetry Update
    Telemetry.setValue("drivetrain/gyro/temperature", M_Gyro.getTemp());
    Telemetry.setValue("drivetrain/gyro/yaw", Yaw);
    Telemetry.setValue("drivetrain/gyro/roll", Roll);
    Telemetry.setValue("drivetrain/gyro/pitch", Pitch);
  }

  @Override
  public void simulationPeriodic() {}
}