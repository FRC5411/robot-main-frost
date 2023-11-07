package frc.lib;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class FrostConfigs {
    public static void configDrive (TalonFX motor) {
        configDrive(motor, DRIVETRAIN.DRIVE_kP, DRIVETRAIN.DRIVE_kF);
    }
    
      // public to avoid warnings
    public static void configAzimuth (TalonFX motor, CANCoder position) {
        configAzimuth(motor, position, DRIVETRAIN.AZIMUTH_kP, DRIVETRAIN.AZIMUTH_kD, DRIVETRAIN.AZIMUTH_kF);
    }
    
    public static void configDrive (TalonFX motor, double kP, double kF) {
        motor.configFactoryDefault();
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
            true, 
            60, 
            60, 
            0));
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
        motor.configSelectedFeedbackCoefficient(360 / (2048 * DRIVETRAIN.AZIMUTH_GEAR_RATIO));
        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
            true, 
            30, 
            40, 
            0.2));
        motor.setSelectedSensorPosition(degreesToFalcon(position.getAbsolutePosition()));
        motor.config_kP(0, kP);
        motor.config_kD(0, kD);
        motor.config_kF(0, kF);
        motor.configNeutralDeadband(DRIVETRAIN.AZIMUTH_DEADBAND);
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

    public static double degreesToFalcon(double degrees) {
        return degrees / (360.0 / ( DRIVETRAIN.AZIMUTH_GEAR_RATIO * 2048.0));
    }

    public static void configShwerve(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setSmartCurrentLimit(60);
        motor.setSecondaryCurrentLimit(60);
        motor.burnFlash();
    }

    public static void configArmMotor(CANSparkMax motor, boolean invert) {
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(invert);
        motor.setSmartCurrentLimit(40);
        motor.setSecondaryCurrentLimit(40);
        motor.burnFlash();
    }
}
