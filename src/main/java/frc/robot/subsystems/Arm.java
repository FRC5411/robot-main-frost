package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Arm extends SubsystemBase {  
  private final CANSparkMax bisceps;  
  private final CANSparkMax elbows;  
  private final RelativeEncoder b_encoder;  
  private final RelativeEncoder e_encoder;  
  private final PIDController arm_pid;  
  private final PIDController elbow_pid;  
  
  public Arm() {    
      bisceps = new CANSparkMax(1, MotorType.kBrushless);    
      elbows = new CANSparkMax(2, MotorType.kBrushless);
      b_encoder = bisceps.getEncoder();    e_encoder = elbows.getEncoder();
      arm_pid = new PIDController(0, 0, 0);    
      elbow_pid = new PIDController(0, 0, 0);
      b_encoder.setPositionConversionFactor(2*Math.PI);    
      e_encoder.setPositionConversionFactor(2*Math.PI);  
  }
  
  public void setarm(double angle) {
      bisceps.set(arm_pid.calculate(b_encoder.getPosition(), angle));  
  }
  public void setelbows(double angle) 
  {    
      elbows.set(elbow_pid.calculate(e_encoder.getPosition(), angle));  
  }
  public void low_score_arm() 
  {
      double l = Constants.ARM.LOW_DIST;
      double no = 0;    
      double whole_angle = Math.acos((500 - Math.pow(l, 2))/(40 * l));    
      double part_angle = Math.atan(no);     double a_angle = whole_angle - part_angle;    
      setarm(a_angle);    
      double e_angle = 180 - (whole_angle + Math.acos((1300-Math.pow(l, 2))/1200));    
      setelbows(e_angle);  
  }
  
  public void high_score() {
      double l = Constants.ARM.HIGH_DIST;
      double no = 0;
      double whole_angle = Math.acos((500 - Math.pow(l, 2))/(40 * l));    
      double part_angle = Math.atan(no);     
      double a_angle = whole_angle - part_angle;    
      setarm(a_angle);    
      double e_angle = 180 - (whole_angle + Math.acos((1300-Math.pow(l, 2))/1200));    
      setelbows(e_angle);  
  }
  
  @Override  public void periodic() {}
  @Override  public void simulationPeriodic() {}
}