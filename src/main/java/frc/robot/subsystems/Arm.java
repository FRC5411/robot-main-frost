package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.lib.Telemetry;

public class Arm extends SubsystemBase 
{
    private final CANSparkMax M_Biscep;
    private final CANSparkMax M_Biscep2;
    private final CANSparkMax M_Elbow;
    private final CANSparkMax M_Elbow2;
    private final CANSparkMax M_Claw;
    private final CANSparkMax M_Claw2;
    private final SparkMaxAbsoluteEncoder Biscep_Encoder;  
    private final SparkMaxAbsoluteEncoder Elbow_Encoder;  
    private final PIDController Biscep_PID;  
    private final PIDController Elbow_PID;
    private final PIDController Claw_PID;

    public Arm() {    
        //Motors
        M_Biscep = new CANSparkMax(1, MotorType.kBrushless);
        M_Biscep2 = new CANSparkMax(1, MotorType.kBrushless);   
        M_Elbow = new CANSparkMax(2, MotorType.kBrushless);
        M_Elbow2 = new CANSparkMax(2, MotorType.kBrushless);
        M_Claw = new CANSparkMax(3, MotorType.kBrushless);
        M_Claw2 = new CANSparkMax(3, MotorType.kBrushless);

        M_Biscep.setIdleMode(IdleMode.kBrake);
        M_Elbow.setIdleMode(IdleMode.kBrake);
        M_Claw.setIdleMode(IdleMode.kBrake);
        M_Biscep2.follow(M_Biscep);
        M_Claw2.follow(M_Claw);
        M_Elbow2.follow(M_Elbow);

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

    @Override  public void periodic() {
        Telemetry.setValue("POP/Biscep/speed", M_Biscep.get());
        Telemetry.setValue("POP/Biscep/temp", M_Biscep.getMotorTemperature());
        Telemetry.setValue("POP/Biscep/voltage", M_Biscep.getAppliedOutput());
        Telemetry.setValue("POP/Biscep/statorcurrent", M_Biscep.getOutputCurrent());
        Telemetry.setValue("POP/Biscep/speed2", M_Biscep2.get());
        Telemetry.setValue("POP/Biscep/temp2", M_Biscep2.getMotorTemperature());
        Telemetry.setValue("POP/Biscep/voltage2", M_Biscep2.getAppliedOutput());
        Telemetry.setValue("POP/Biscpe/statorcurrent2", M_Biscep2.getOutputCurrent());
        Telemetry.setValue("POP/Elbow/speed", M_Elbow.get());
        Telemetry.setValue("POP/Elbow/temp", M_Elbow.getMotorTemperature());
        Telemetry.setValue("POP/Elbow/voltage", M_Elbow.getAppliedOutput());
        Telemetry.setValue("POP/Elbow/statorcurrent", M_Elbow.getOutputCurrent());
        Telemetry.setValue("POP/Elbow/speed2", M_Elbow2.get());
        Telemetry.setValue("POP/Elbow/temp2", M_Elbow2.getMotorTemperature());
        Telemetry.setValue("POP/Elbow/voltage2", M_Elbow2.getAppliedOutput());
        Telemetry.setValue("POP/Elbow/statorcurrent2", M_Elbow2.getOutputCurrent());
        Telemetry.setValue("POP/Claw/speed", M_Claw.get());
        Telemetry.setValue("POP/Claw/temp", M_Claw.getMotorTemperature());
        Telemetry.setValue("POP/Claw/voltage", M_Claw.getAppliedOutput());
        Telemetry.setValue("POP/Claw/statorcurrent", M_Claw.getOutputCurrent());
        Telemetry.setValue("POP/Claw/speed2", M_Claw2.get());
        Telemetry.setValue("POP/Claw/temp2", M_Claw2.getMotorTemperature());
        Telemetry.setValue("POP/Claw/voltage2", M_Claw2.getAppliedOutput());
        Telemetry.setValue("POP/Claw/statorcurrent2", M_Claw2.getOutputCurrent());
    }
    
    @Override  public void simulationPeriodic() {}
}