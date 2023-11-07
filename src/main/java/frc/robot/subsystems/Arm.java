package frc.robot.subsystems;
import static frc.robot.Constants.ARM.*;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.ArmPosition;
import frc.lib.ButtonBoard;
import frc.lib.FrostConfigs;
import frc.lib.Telemetry;

import frc.robot.Constants.ARM.positions;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {
    private CANSparkMax m_stage1;
    private CANSparkMax m_stage2;
    private CANSparkMax m_stage3;
    private DutyCycleEncoder m_stage1Encoder;  
    private DutyCycleEncoder m_stage2Encoder;
    private DutyCycleEncoder m_stage3Encoder;
    private ProfiledPIDController m_stage1PID;
    private ProfiledPIDController m_stage2PID;
    private ProfiledPIDController m_stage3PID;
    private PinchersofPower m_clawSubsystem;
    private ButtonBoard m_copilotController;
    private double m_stage1Target = idlePosition.getStage1Angle();
    private double m_stage2Target = idlePosition.getStage2Angle();
    private double m_stage3Target = idlePosition.getStage3Angle();
    // manual targets actually unused
    private double m_manualTargetX = 0;
    private double m_manualTargetY = 0;
    private double m_manualTargetTheta = 0;
    private HashMap<positions, ArmPosition> positionMap = new HashMap<positions, ArmPosition>();
    private boolean movingToIdle = false;
    public positions target = positions.Idle;
    private ArmFeedforward m_stage1FF;
    private ArmFeedforward m_stage2FF;
    private ArmFeedforward m_stage3FF;

    private double tempThetaSpeed = THETA_SPEED;

    private double stage1 = 0;
    private double stage2 = 0;
    private double stage3 = 0;

    public Arm(PinchersofPower m_claw, ButtonBoard copilotController) {
        positionMap.put(positions.ScoreHighCone, scoreHighConePosition);
        positionMap.put(positions.ScoreMidCone, scoreMidConePosition);
        positionMap.put(positions.ScoreHighCube, scoreHighCubePosition);
        positionMap.put(positions.ScoreMidCube, scoreMidCubePosition);
        positionMap.put(positions.ScoreLow, scoreLowPosition);
        positionMap.put(positions.Floor, floorPosition);
        positionMap.put(positions.AutonFloor, autonFloorPosition);
        positionMap.put(positions.FloorAlt, floorAltPosition);
        positionMap.put(positions.Substation, substationPosition);
        positionMap.put(positions.Idle, idlePosition);
        positionMap.put(positions.DipHighCone, dipHighConePosition);
        positionMap.put(positions.DipMidCone, dipMidConePosition);

        m_clawSubsystem = m_claw;
        m_copilotController = copilotController;

        m_stage1 = new CANSparkMax(CAN.ARM_STAGE_1_ID, MotorType.kBrushless);
        m_stage2 = new CANSparkMax(CAN.ARM_STAGE_2_ID, MotorType.kBrushless);
        m_stage3 = new CANSparkMax(CAN.ARM_STAGE_3_ID, MotorType.kBrushless);

        FrostConfigs.configArmMotor(m_stage1, false);
        FrostConfigs.configArmMotor(m_stage2, false);
        FrostConfigs.configArmMotor(m_stage3, false);

        m_stage1Encoder = new DutyCycleEncoder(DIO.ARM_STAGE_1_ENCODER_ID);
        m_stage2Encoder = new DutyCycleEncoder(DIO.ARM_STAGE_2_ENCODER_ID);
        m_stage3Encoder = new DutyCycleEncoder(DIO.ARM_STAGE_3_ENCODER_ID);

        m_stage1FF = new ArmFeedforward(STAGE_1_Ks, STAGE_1_Kg, 0,0); // 1.37 1.35 1.3
        m_stage2FF = new ArmFeedforward(STAGE_2_Ks, STAGE_2_Kg, 0,0);
        m_stage3FF = new ArmFeedforward(STAGE_3_Ks, STAGE_3_Kg, 0,0);

        m_stage1PID = new ProfiledPIDController(STAGE_1_Kp, STAGE_1_Ki, STAGE_1_Kd, STAGE_1_CONSTRAINTS);
        m_stage2PID = new ProfiledPIDController(STAGE_2_Kp, STAGE_2_Ki, STAGE_2_Kd, STAGE_2_CONSTRAINTS);
        m_stage3PID = new ProfiledPIDController(STAGE_3_Kp, STAGE_3_Ki, STAGE_3_Kd, STAGE_3_CONSTRAINTS);

        m_stage1PID.enableContinuousInput(0, 360);
        m_stage2PID.enableContinuousInput(0, 360);
        m_stage3PID.enableContinuousInput(0, 360);

        m_stage1PID.setTolerance(3.0);//0
        m_stage2PID.setTolerance(3.0);//3//2//1//0
        m_stage3PID.setTolerance(3.0);

        resetProfiles();

        m_copilotController.button(10).whileTrue(new RepeatCommand( new InstantCommand(() -> {
            if (m_copilotController.getRawButton(9)) {
                m_manualTargetTheta += THETA_SPEED;
                stage3 += THETA_SPEED;
            }
        })));
        m_copilotController.button(11).whileTrue(new RepeatCommand( new InstantCommand(() -> {
            if (m_copilotController.getRawButton(9)) {
                m_manualTargetTheta -= THETA_SPEED;
                stage3 -= THETA_SPEED;
            }
        })));
    }

    private double[] inverseKinematics (double x, double y, double theta) {
        double[] output = new double[3];

        double l1 = STAGE_1_LENGTH;
        double l2 = STAGE_2_LENGTH;
        double l3 = Math.hypot(x, y);
        double thetaA = Math.toDegrees(Math.acos((l2*l2-l1*l1-l3*l3)/(-2*l1*l3)));
        double thetaB = Math.toDegrees(Math.acos((l3*l3-l1*l1-l2*l2)/(-2*l1*l2)));

        output[0] = (360 + Math.toDegrees(Math.atan2(y, x)) + thetaA) % 360;
        output[1] = (360 + output[0] + thetaB + 180) % 360;
        output[2] = (360 + theta) % 360;

        if ( y >= 20 || x > 49.0 || Double.isNaN(output[0]) || 
            Double.isNaN(output[1]) || Double.isNaN(output[2])) output = getCurrentPoint();

        return output;
    }

    private void moveToPoint (double x, double y, double theta) {
        stage1 += m_copilotController.getJoystick().getY()*tempThetaSpeed;
        stage2 += m_copilotController.getJoystick().getX()*tempThetaSpeed;
        if (tempThetaSpeed != THETA_SPEED && m_copilotController.getJoystick().getNorm() != 0) tempThetaSpeed = THETA_SPEED;
        moveToAngles(stage1, stage2, stage3);
    }

    private double[] forwardKinematics ( double stage1Degrees, double stage2Degrees, double stage3Degrees ) {
        double[] output = new double[3];
        output[0] = Math.cos(Math.toRadians(stage1Degrees)) * STAGE_1_LENGTH + Math.cos(Math.toRadians(stage2Degrees)) * (STAGE_2_LENGTH);
        output[1] = Math.sin(Math.toRadians(stage1Degrees)) * STAGE_1_LENGTH + Math.sin(Math.toRadians(stage2Degrees)) * (STAGE_2_LENGTH);
        output[2] = (360 + stage3Degrees) % 360;
        return output;
    }

    private double[] getCurrentPoint () {
        return forwardKinematics(m_stage1Encoder.getAbsolutePosition()*360 - STAGE_1_OFFSET, m_stage2Encoder.getAbsolutePosition()*360-STAGE_2_OFFSET, m_stage3Encoder.getAbsolutePosition()*360-STAGE_3_OFFSET);
    }

    private void moveToAngles (double stage1Angle, double stage2Angle, double stage3Angle) {
        m_manualTargetX = forwardKinematics((stage1Angle - STAGE_1_OFFSET)%360, (stage2Angle - STAGE_2_OFFSET)%360, stage3Angle - STAGE_3_OFFSET)[0];
        m_manualTargetY = forwardKinematics((stage1Angle - STAGE_1_OFFSET)%360, (stage2Angle - STAGE_2_OFFSET)%360, stage3Angle - STAGE_3_OFFSET)[1];
        m_manualTargetTheta = stage3Angle - STAGE_3_OFFSET;
        setStage1Target(stage1Angle % 360);
        setStage2Target(stage2Angle % 360);
        setStage3Target(stage3Angle % 360);
    }

    private void moveToPositionInit (positions position) {
        if(!(position == positions.DipHighCone) || (position == positions.DipMidCone)) resetProfiles();
        RobotContainer.setCopilotLEDs();
        movingToIdle = m_clawSubsystem.setClawState(position);
    }

    private void moveToPositionExecute (positions position) {
        target = position;
        ArmPosition internalTarget = positionMap.get(position);
        if (!movingToIdle || (movingToIdle &&
            Math.abs(m_stage1Encoder.getAbsolutePosition()*360 - m_stage1Target) < JOINT_ANGLE_DEADZONE &&
            Math.abs(m_stage2Encoder.getAbsolutePosition()*360 - m_stage2Target) < JOINT_ANGLE_DEADZONE)) {
                moveToAngles(internalTarget.getStage1Angle(), internalTarget.getStage2Angle(), internalTarget.getStage3Angle());
        } else {
            moveToAngles(internalTarget.getStage1Angle(), internalTarget.getStage2Angle(), m_stage3Encoder.getAbsolutePosition()*360);   
        }
    }

    public Boolean isAtTarget () {
            return (
                Math.abs(m_stage1Encoder.getAbsolutePosition()*360 - m_stage1Target) < JOINT_ANGLE_DEADZONE &&
                Math.abs(m_stage2Encoder.getAbsolutePosition()*360 - m_stage2Target) < JOINT_ANGLE_DEADZONE &&
                Math.abs(m_stage3Encoder.getAbsolutePosition()*360 - m_stage3Target) < JOINT_ANGLE_DEADZONE );
    }

    public void setStage1Target(double angle) {
        m_stage1Target = angle;
    }

    public void setStage2Target(double angle) {
        m_stage2Target = angle;
    }

    public void setStage3Target(double angle) {
        m_stage3Target = angle;
    }

    public Command goToScoreHigh(){
        if(m_clawSubsystem.wantCone()){
            return new SequentialCommandGroup(moveToPositionCommand(positions.ScoreHighCone).withTimeout(0.6), goToDipHigh());
        }
        else{
            return moveToPositionCommand(positions.ScoreHighCube);
        }
    }

    public Command goToDipHigh() {
        return moveToPositionCommand(positions.DipHighCone);
    }

    public Command goToScoreMid(){
        if(m_clawSubsystem.wantCone()){
            return new SequentialCommandGroup(moveToPositionCommand(positions.ScoreMidCone).withTimeout(0.6), goToDipMid());//0.85 sec
        }
        else{
            return moveToPositionCommand(positions.ScoreMidCube);
        }
    }

    public Command goToDipMid() {
        return moveToPositionCommand(positions.DipMidCone);
    }

    public Command moveToPositionCommand (positions position) {
        return new FunctionalCommand(
            () -> moveToPositionInit(position), 
            () -> moveToPositionExecute(position), 
            interrupted -> { movingToIdle = false; },
            () -> false,
            this
        );
    }

    public Command moveToPositionTerminatingCommand( positions position ) {
        return new FunctionalCommand(
            () -> moveToPositionInit(position), 
            () -> moveToPositionExecute(position), 
            interrupted -> { movingToIdle = false; },
            this::isAtTarget,
            this
        );
    }

    public Command moveToPointCommand () {
        return new FunctionalCommand(
            () -> RobotContainer.setCopilotLEDs(), 
            () -> {
                m_manualTargetX += m_copilotController.getJoystick().getX() * X_SPEED;
                m_manualTargetY += m_copilotController.getJoystick().getY() * Y_SPEED;
                moveToPoint(m_manualTargetX, m_manualTargetY, m_manualTargetTheta);
            },
            interrupted -> {},
            () -> true,
            this
        );
    }

    public Command onManual () {
        return new InstantCommand( () -> {
            tempThetaSpeed = 3*THETA_SPEED;
            stage1 = m_stage1Encoder.getAbsolutePosition()*360;
            stage2 = m_stage2Encoder.getAbsolutePosition()*360;
            stage3 = m_stage3Encoder.getAbsolutePosition()*360;
            m_manualTargetX = getCurrentPoint()[0];
            m_manualTargetY = getCurrentPoint()[1];
            m_manualTargetTheta = getCurrentPoint()[2];
        });
    }

    public Command defaultCommand () {
        return new FunctionalCommand(
            () -> {},
            () -> {
                if (RobotContainer.isManual()) moveToPointCommand().schedule();
                else {
                    if (DriverStation.isAutonomous()) return;
                    moveToPositionCommand(positions.Idle).schedule();
                }
            }, 
            (interrupted) -> {},
            () -> false, 
            this
        );
    }

    @Override  
    public void periodic() {
        Telemetry.setValue("Arm/targetPosition", target.name());
        Telemetry.setValue("Arm/currentPoint/x", getCurrentPoint()[0]);
        Telemetry.setValue("Arm/currentPoint/y", getCurrentPoint()[1]);
        Telemetry.setValue("Arm/currentPoint/theta", getCurrentPoint()[2]);
        Telemetry.setValue("Arm/manualTarget/manualTargetX", m_manualTargetX);
        Telemetry.setValue("Arm/manualTarget/manualTargetY", m_manualTargetY);
        Telemetry.setValue("Arm/manualTarget/manualTargetTheta", m_manualTargetTheta);
        double[] telemetryConversion = inverseKinematics(m_manualTargetX, m_manualTargetY, m_manualTargetTheta);
        Telemetry.setValue("Arm/manualTarget/manualTargetFK", forwardKinematics(telemetryConversion[0], telemetryConversion[1], telemetryConversion[2]));
        Telemetry.armTelemetry(m_stage1, m_stage1Encoder, STAGE_1_OFFSET, "stage1");
        Telemetry.setValue("Arm/stage1/targetPosition", m_stage1Target);
        Telemetry.armTelemetry(m_stage2, m_stage2Encoder, STAGE_2_OFFSET, "stage2");
        Telemetry.setValue("Arm/stage2/targetPosition", m_stage2Target);
        Telemetry.armTelemetry(m_stage3, m_stage3Encoder, STAGE_3_OFFSET, "stage3");
        Telemetry.setValue("Arm/stage3/targetPosition", m_stage3Target);

        if ( DriverStation.isEnabled() || DriverStation.isAutonomousEnabled() ) {
            double stage1Calc = 
                MathUtil.clamp(
                    m_stage1FF.calculate(
                        Math.toRadians(m_stage1PID.getSetpoint().position), 
                        m_stage1PID.getSetpoint().velocity) + 
                    12.0 * m_stage1PID.calculate( 
                        m_stage1Encoder.getAbsolutePosition() * 360 - STAGE_1_OFFSET, 
                        m_stage1Target - STAGE_1_OFFSET), -12, 12);
            
            double stage2Calc = 
                MathUtil.clamp(
                    m_stage2FF.calculate(
                        Math.toRadians(
                            m_stage1PID.getSetpoint().position), 
                            m_stage2PID.getSetpoint().velocity) + 
                    12.0 * m_stage2PID.calculate(
                        m_stage2Encoder.getAbsolutePosition() * 360 - STAGE_2_OFFSET, 
                        m_stage2Target - STAGE_2_OFFSET), -12, 12);

            double stage3Calc = 
                MathUtil.clamp(
                    m_stage3FF.calculate(
                        Math.toRadians( m_stage1PID.getSetpoint().position ), 
                        m_stage2PID.getSetpoint().velocity ) + 
                    12.0 * m_stage3PID.calculate(
                        m_stage3Encoder.getAbsolutePosition() * 360 - STAGE_3_OFFSET, 
                        m_stage3Target - STAGE_3_OFFSET), -12, 12);

            Telemetry.setValue("Arm/stage1/PIDVOLT", stage1Calc);
            Telemetry.setValue("Arm/stage2/PIDVOLT", stage2Calc);
            Telemetry.setValue("Arm/stage3/PIDVOLT", stage3Calc);

            m_stage1.setVoltage(stage1Calc);
            m_stage2.setVoltage(stage2Calc);
            m_stage3.setVoltage(stage3Calc);
        } else {
            // prevent CAN timeouts when disabled, actual motor stoppage is handled at a lower level
            m_stage1.set(0);
            m_stage2.set(0);
            m_stage3.set(0);
            resetProfiles();
        }
    }
    
    @Override  
    public void simulationPeriodic() {}

    public void resetProfiles() {
        m_stage1PID.reset(m_stage1Encoder.getAbsolutePosition() * 360 - STAGE_1_OFFSET);
        m_stage2PID.reset(m_stage2Encoder.getAbsolutePosition() * 360 - STAGE_2_OFFSET);
        m_stage3PID.reset(m_stage3Encoder.getAbsolutePosition() * 360 - STAGE_3_OFFSET);
    }
}