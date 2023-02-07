package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.triangle;
import frc.robot.Constants.*;
import frc.lib.Telemetry;
public class Arm extends SubsystemBase {
    private final CANSparkMax M_Biscep;
    private final CANSparkMax M_Biscep2;
    private final CANSparkMax M_Elbow;
    private final CANSparkMax M_Elbow2;
    private final CANSparkMax M_Claw;
    private final CANSparkMax M_Claw2;
    private final SparkMaxAbsoluteEncoder Biscep_Encoder;  
    private final SparkMaxAbsoluteEncoder Elbow_Encoder;
    private final SparkMaxAbsoluteEncoder Claw_Encoder;
    private final ProfiledPIDController Biscep_PID;  
    private final ProfiledPIDController Elbow_PID;
    private final ProfiledPIDController Claw_PID;
    private final ArmFeedforward FBiscep;
    private final ArmFeedforward FElbow;
    private final ArmFeedforward FClaw;
    private final TrapezoidProfile.Constraints BiscepProfile;
    private final TrapezoidProfile.Constraints ElbowProfile;
    private final TrapezoidProfile.Constraints ClawProfile;

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
        M_Biscep2.setIdleMode(IdleMode.kBrake);
        M_Elbow2.setIdleMode(IdleMode.kBrake);
        M_Claw2.setIdleMode(IdleMode.kBrake);
        M_Biscep2.follow(M_Biscep);
        M_Claw2.follow(M_Claw);
        M_Elbow2.follow(M_Elbow);

        Biscep_Encoder = M_Biscep.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        Elbow_Encoder = M_Elbow.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        Claw_Encoder = M_Claw.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        Biscep_Encoder.setPositionConversionFactor(360);
        Elbow_Encoder.setPositionConversionFactor(360);
        Claw_Encoder.setPositionConversionFactor(360);

        BiscepProfile = new TrapezoidProfile.Constraints(0, 0);
        ElbowProfile = new TrapezoidProfile.Constraints(0, 0);
        ClawProfile = new TrapezoidProfile.Constraints(0, 0);

        FBiscep = new ArmFeedforward(0, 0, 0, 0);
        FElbow = new ArmFeedforward(0, 0, 0, 0);
        FClaw = new ArmFeedforward(0, 0, 0, 0);

        Biscep_PID = new ProfiledPIDController(0, 0, 0, BiscepProfile);    
        Elbow_PID = new ProfiledPIDController(0, 0, 0, ElbowProfile);
        Claw_PID = new ProfiledPIDController(0, 0, 0, ClawProfile);
    }

    public void movetopoint(double x, double y, double claw) {
        triangle Triangle = new triangle(x, y, Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
        double angle1 = Triangle.getangleA() + (90 - Math.atan2(x, y));
        double angle2 = Triangle.getangleB();
        posArm(angle1);
        posElbows(angle2);
        posClaws(claw);
    }

    public void setArm(double speed) {
        M_Biscep.set(speed);
    }

    public void setElbows(double speed) {    
        M_Elbow.set(speed);  
    }

    public void setClaws(double speed) {    
        M_Claw.set(speed);  
    }

    public void posArm(double angle) {
        M_Biscep.set(Biscep_PID.calculate(Biscep_Encoder.getPosition(), angle) + FBiscep.calculate(Biscep_Encoder.getPosition(), Biscep_Encoder.getVelocity()));  
    }

    public void posElbows(double angle) {
        M_Elbow.set(Elbow_PID.calculate(Elbow_Encoder.getPosition(), angle) + FElbow.calculate(Elbow_Encoder.getPosition(), Elbow_Encoder.getVelocity()));  
    }

    public void posClaws(double angle) {
        M_Claw.set(Claw_PID.calculate(Claw_Encoder.getPosition(), angle) + FClaw.calculate(Claw_Encoder.getPosition(), Claw_Encoder.getVelocity()));  
    }

    public void lowArmScore() {
        posArm(ARM.LOW_ARM_ANG);
        posElbows(ARM.LOW_ELBOW_ANG);  
        posClaws(ARM.LOW_CLAW_ANG);
    }

    public void highArmScore() {
        posArm(ARM.HIGH_ARM_ANG);
        posElbows(ARM.HIGH_ELBOW_ANG);  
        posClaws(ARM.HIGH_CLAW_ANG);
    }
    
    public void idleArmScore() {
        posArm(ARM.IDLE_ARM_ANG);
        posElbows(ARM.IDLE_ELBOW_ANG);  
        posClaws(ARM.IDLE_CLAW_ANG);
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