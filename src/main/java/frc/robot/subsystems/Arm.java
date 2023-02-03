//Root Package
package frc.robot.subsystems;

//Libraries
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Arm extends SubsystemBase 
{
    private final CANSparkMax M_Biscep;  
    private final CANSparkMax M_Elbow;
    private final CANSparkMax M_Claw;
    private final SparkMaxAbsoluteEncoder Biscep_Encoder;  
    private final SparkMaxAbsoluteEncoder Elbow_Encoder;  
    private final PIDController Biscep_PID;  
    private final PIDController Elbow_PID;
    private final PIDController Claw_PID;

    public Arm() {    
        //Motors
        M_Biscep = new CANSparkMax(1, MotorType.kBrushless);    
        M_Elbow = new CANSparkMax(2, MotorType.kBrushless);
        M_Claw = new CANSparkMax(3, MotorType.kBrushless);

        M_Biscep.setIdleMode(IdleMode.kBrake);
        M_Elbow.setIdleMode(IdleMode.kBrake);

        Biscep_Encoder = M_Biscep.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        Elbow_Encoder = M_Elbow.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        Biscep_Encoder.setPositionConversionFactor(2*Math.PI);
        Elbow_Encoder.setPositionConversionFactor(2*Math.PI);

        Biscep_PID = new PIDController(0, 0, 0);    
        Elbow_PID = new PIDController(0, 0, 0);
        Claw_PID = new PIDController(0, 0, 0);
    }

    public void setArm(double angle) {
        M_Biscep.set(Biscep_PID.calculate(Biscep_Encoder.getPosition(), angle));  
    }

    public void setElbows(double angle) {    
        M_Elbow.set(Elbow_PID.calculate(Elbow_Encoder.getPosition(), angle));  
    }

    public void setClaws(double angle) {    
        M_Claw.set(Claw_PID.calculate(Elbow_Encoder.getPosition(), angle));  
    }

    public void lowArmScore() {
        setArm(ARM.LOW_ARM_ANG);
        setElbows(ARM.LOW_ELBOW_ANG);  
        setClaws(ARM.LOW_CLAW_ANG);
    }

    public void highArmScore() {
        setArm(ARM.HIGH_ARM_ANG);
        setElbows(ARM.HIGH_ELBOW_ANG);  
        setClaws(ARM.HIGH_CLAW_ANG);
    }
    
    public void idleArmScore() {
        setArm(ARM.IDLE_ARM_ANG);
        setElbows(ARM.IDLE_ELBOW_ANG);  
        setClaws(ARM.IDLE_CLAW_ANG);
    }

    @Override  public void periodic() {}
    
    @Override  public void simulationPeriodic() {}
}