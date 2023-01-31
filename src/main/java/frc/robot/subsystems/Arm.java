//Root Package
package frc.robot.subsystems;

//Libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

/**
 * Arm Subsystem
 */
public class Arm extends SubsystemBase 
{
    //Instance Variables
    private final CANSparkMax M_Biscep;  
    private final CANSparkMax M_Elbow;  
    private final RelativeEncoder Biscep_Encoder;  
    private final RelativeEncoder Elbow_Encoder;  
    private final PIDController Biscep_PID;  
    private final PIDController Elbow_PID;  

    //Arm Subsystem Constructor
    public Arm() 
    {    
        //Motors
        M_Biscep = new CANSparkMax(1, MotorType.kBrushless);    
        M_Elbow = new CANSparkMax(2, MotorType.kBrushless);

        M_Biscep.setIdleMode(IdleMode.kBrake);
        M_Elbow.setIdleMode(IdleMode.kBrake);

        Biscep_Encoder = M_Biscep.getEncoder();
        Elbow_Encoder = M_Elbow.getEncoder();

        Biscep_Encoder.setPositionConversionFactor(2*Math.PI);
        Elbow_Encoder.setPositionConversionFactor(2*Math.PI);

        Biscep_PID = new PIDController(0, 0, 0);    
        Elbow_PID = new PIDController(0, 0, 0);
    }

    /**
     * Set Arm to given angle utilizing onbaord Encoders
     *  @param angle - Arm Angle
     */
    public void setArm(double angle) 
    {
        M_Biscep.set(Biscep_PID.calculate(Biscep_Encoder.getPosition(), angle));  
    }

    /**
     * Set Elbows to given angle utilizing onbaord Encoders
     *  @param angle - Elbow Angle
     */
    public void setElbows(double angle) 
    {    
        M_Elbow.set(Elbow_PID.calculate(Elbow_Encoder.getPosition(), angle));  
    }
    /**
     * Position Arm for Low Row Score
     */
    public void lowArmScore() 
    {
        double l = ARM.LOW_DIST;
        double no = 0;    
        double whole_angle = Math.acos((500 - Math.pow(l, 2))/(40 * l));    
        double part_angle = Math.atan(no);     double a_angle = whole_angle - part_angle;    
        setArm(a_angle);    
        double e_angle = 180 - (part_angle + Math.acos((1300-Math.pow(l, 2))/1200));    
        setElbows(e_angle);  
    }
    /**
     * Position Arm for High Row Score
     */
    public void highArmScore() 
    {
        double l = ARM.HIGH_DIST;
        double donthaveconstants = 0;
        double whole_angle = Math.acos((500 - Math.pow(l, 2))/(40 * l));    
        double part_angle = Math.atan(donthaveconstants);     
        double a_angle = whole_angle - part_angle;    
        setArm(a_angle);    
        double e_angle = 180 - (whole_angle + Math.acos((1300-Math.pow(l, 2))/1200));    
        setElbows(e_angle);  
    }
    
    //Subsystem Perioid Method
    @Override  public void periodic() {}
    
    //Subsystem Simulation Perioid Method
    @Override  public void simulationPeriodic() {}
}