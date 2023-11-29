package frc.lib;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

import static frc.robot.Constants.DRIVETRAIN.*;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;



public class FrostConfigs {
    public static void configDrive (TalonFX motor) {
        configDrive(motor, DRIVE_kP, DRIVE_kF);
    }
    
      // public to avoid warnings
    public static void configAzimuth (TalonFX motor, CANCoder position) {
        configAzimuth(motor, position, AZIMUTH_kP, AZIMUTH_kD, AZIMUTH_kF);
    }
    
    public static void configDrive (TalonFX motor, double kP, double kF) {
        motor.configFactoryDefault();
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        motor.setSelectedSensorPosition(0);
        motor.config_kP(0, kP);
        motor.config_kF(0, kF);
        motor.configVoltageCompSaturation(12);
        motor.enableVoltageCompensation(true);
    }
    
    public static void configAzimuth (TalonFX motor, CANCoder position, double kP, double kD, double kF) {
        motor.configFactoryDefault();
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configRemoteFeedbackFilter(position, 0);
        motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        motor.configSelectedFeedbackCoefficient(360 / (2048 * AZIMUTH_GEAR_RATIO));
        motor.configStatorCurrentLimit(AZIMUTH_CURRENT_LIMIT);
        motor.setSelectedSensorPosition(SimpleUtils.degreesToFalcon(position.getAbsolutePosition()));
        motor.config_kP(0, kP);
        motor.config_kD(0, kD);
        motor.config_kF(0, kF);
        motor.configNeutralDeadband(AZIMUTH_DEADBAND);
    }
    
    public static void configPosition (CANCoder encoder, double offset) {
        encoder.configFactoryDefault();
        encoder.configMagnetOffset(offset);
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.setPositionToAbsolute();
    }
    
    public static void configPID(PIDController controller) {
        controller.enableContinuousInput(0, 360);
        controller.setTolerance(0);
    }

    public static void configShwerve(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setSmartCurrentLimit(60);
        motor.setSecondaryCurrentLimit(60);
        motor.burnFlash();
    }
}
