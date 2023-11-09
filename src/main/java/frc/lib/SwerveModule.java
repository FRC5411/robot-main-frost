package frc.lib;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.MathUtil;
import static frc.robot.Constants.DRIVETRAIN.*;

public class SwerveModule {
    public TalonFX driveMotor;
    public TalonFX angleMotor;
    public CANCoder angleEncoder;
    public PIDController angleController;
    public double kF;
    public SwerveModuleState desiredState = new SwerveModuleState();
    public String key;

    public SwerveModule(TalonFX driveMotor, TalonFX angleMotor, CANCoder angleEncoder, 
    PIDController angleController, double kF, String key) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.angleController = angleController;
        this.kF = kF;
        this.key = key;
    }

    public SwerveModule(int driveID, int angleID, int angleEncoderID,
    double offset, PIDController angleController, double kF, String key) {
        this(
            new TalonFX(driveID, "drivetrain"), 
            new TalonFX(angleID, "drivetrain"), 
            new CANCoder(angleEncoderID, "drivetrain"), 
            angleController, 
            kF, 
            key);
        FrostConfigs.configDrive(driveMotor);
        FrostConfigs.configAzimuth(angleMotor, angleEncoder);
        FrostConfigs.configPosition(angleEncoder, offset);
        FrostConfigs.configAzimuthPID(angleController);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
        setAngle(desiredState);
        setVelocity(desiredState);
    }

    public void setLockedState(SwerveModuleState lockedState) {
        this.desiredState = lockedState;
        lockedState = SwerveModuleState.optimize(lockedState, Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
        lockAngle(lockedState);
        setVelocity(lockedState);
    }

    public void setAngle(SwerveModuleState desiredState) {
        double angleDegrees = 
            Math.abs(desiredState.speedMetersPerSecond) <= (MAX_LINEAR_SPEED * 0.01) 
            ? angleEncoder.getAbsolutePosition() : desiredState.angle.getDegrees();

        angleMotor.set(
            ControlMode.PercentOutput,
            MathUtil.clamp(
                angleController.calculate(angleEncoder.getAbsolutePosition(), angleDegrees) 
                + kF * Math.signum(angleController.getPositionError()), -1, 1));
        // angleMotor.set(ControlMode.Position, angleDegrees);
    }

    public void setVelocity(SwerveModuleState desiredState) {
        driveMotor.set(
            ControlMode.Velocity, 
            (desiredState.speedMetersPerSecond * DRIVE_GEAR_RATIO
            / (Math.PI * WHEEL_DIAMETER_METERS) * 2048) / 10);
    }

    public void lockAngle(SwerveModuleState desiredState) {
        angleMotor.set(
            ControlMode.PercentOutput, 
            angleController.calculate(angleEncoder.getAbsolutePosition(), desiredState.angle.getDegrees()) 
            + kF * Math.signum(angleController.getPositionError()));
    }

    public void stopMotors() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        angleMotor.set(ControlMode.PercentOutput, 0);
    }

    public SwerveModuleState geDesiredState() {
        return desiredState;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            (driveMotor.getSelectedSensorVelocity() * 10 * WHEEL_PERIMETER_METERS)
            / (2048 * DRIVE_GEAR_RATIO), 
            Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            (driveMotor.getSelectedSensorPosition() * WHEEL_PERIMETER_METERS)
            / (2048 * DRIVE_GEAR_RATIO), 
            Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
    }

    public void telemetry() {
        Telemetry.setValue("drivetrain/modules/"+key+"/azimuth/targetPosition", geDesiredState().angle.getDegrees() % 360);
        Telemetry.setValue("drivetrain/modules/"+key+"/drive/targetSpeed", geDesiredState().speedMetersPerSecond);
        Telemetry.setValue("drivetrain/modules/"+key+"/azimuth/actualPosition", getState().angle.getDegrees());
        Telemetry.setValue("drivetrain/modules/"+key+"/azimuth/SelectedSensorPos", angleMotor.getSelectedSensorPosition());
        Telemetry.setValue("drivetrain/modules"+key+"/azimuth/targetPositionTicks", geDesiredState().angle.getDegrees() % 360 * 2048 * AZIMUTH_GEAR_RATIO / 360);
        Telemetry.setValue("drivetrain/modules/"+key+"/drive/actualSpeed", getState().speedMetersPerSecond);
        Telemetry.setValue("drivetrain/modules/"+key+"/azimuth/outputVoltage", angleMotor.getMotorOutputVoltage());
        Telemetry.setValue("drivetrain/modules/"+key+"/drive/outputVoltage", driveMotor.getMotorOutputVoltage());
        Telemetry.setValue("drivetrain/modules/"+key+"/azimuth/statorCurrent", angleMotor.getStatorCurrent());
        Telemetry.setValue("drivetrain/modules/"+key+"/drive/statorCurrent", driveMotor.getStatorCurrent());
        Telemetry.setValue("drivetrain/modules/"+key+"/drive/Temperature", driveMotor.getTemperature());
        Telemetry.setValue("drivetrain/modules/"+key+"/azimuth/Temperature", driveMotor.getTemperature());
    }

    public static double degreesToFalcon(double degrees) {
        return degrees / (360.0 / ( AZIMUTH_GEAR_RATIO * 2048.0));
    }
}