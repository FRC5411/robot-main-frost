package frc.lib;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.MathUtil;
import static frc.robot.Constants.DRIVETRAIN.*;

public class SwerveModule {
    public WPI_TalonFX driveMotor;
    public WPI_TalonFX angleMotor;
    public WPI_CANCoder angleEncoder;
    public PIDController angleController;
    public double kF;
    public SwerveModuleState desiredState = new SwerveModuleState();
    public String key;

    public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX angleMotor, WPI_CANCoder angleEncoder, 
                        PIDController angleController, double kF, String key) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.angleController = angleController;
        this.kF = kF;
        this.key = key;
    }

    public SwerveModule(int driveID, int anlgeID, int cancoderID, PIDController angleController, 
                        double kF, double offset, String key) {
        this(new WPI_TalonFX(driveID), new WPI_TalonFX(anlgeID), new WPI_CANCoder(cancoderID), angleController, kF, key);
        FrostConfigs.configPID(angleController);
        FrostConfigs.configPosition(angleEncoder, offset);
        FrostConfigs.configDrive(this.driveMotor);
        FrostConfigs.configAzimuth(
            this.angleMotor, angleEncoder, 
            angleController.getP(), angleController.getD(), kF);
    }

    public void setDesiredState( SwerveModuleState desiredState ) {
        this.desiredState = desiredState;
        desiredState = SwerveModuleState.optimize(desiredState, getAzimuthRotation());
        setAngle(desiredState);
        setVelocity(desiredState);
    }

    public void setLockedState( SwerveModuleState desiredState) {
        this.desiredState = desiredState;
        desiredState = SwerveModuleState.optimize(desiredState, getAzimuthRotation());
        lockAngle(desiredState);
        setVelocity(desiredState);
    }

    public void setAngle ( SwerveModuleState desiredState ) {
        double angleDegrees = 
            Math.abs(desiredState.speedMetersPerSecond) <= (MAX_LINEAR_SPEED * 0.01) 
            ? getAzimuthRotation().getDegrees() : desiredState.angle.getDegrees();

        angleMotor.set(
            ControlMode.PercentOutput,
            MathUtil.clamp(
                angleController.calculate( getAzimuthRotation().getDegrees(), angleDegrees ) 
                + kF * Math.signum(angleController.getPositionError()), -1, 1));
        // angleMotor.set(ControlMode.Position, angleDegrees);
    }

    public void setVelocity( SwerveModuleState desiredState ) {
        driveMotor.set(
            ControlMode.Velocity, 
            ( desiredState.speedMetersPerSecond * DRIVE_GEAR_RATIO
            / ( Math.PI * WHEEL_DIAMETER_METERS ) * 2048 ) / 10 );
    }

    public void lockAngle ( SwerveModuleState desiredState ) {
        angleMotor.set(
            ControlMode.PercentOutput, 
            angleController.calculate( getAzimuthRotation().getDegrees(), desiredState.angle.getDegrees() ) 
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
            / (2048 * DRIVE_GEAR_RATIO), getAzimuthRotation() );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            (driveMotor.getSelectedSensorPosition() * WHEEL_PERIMETER_METERS)
            / (2048 * DRIVE_GEAR_RATIO), getAzimuthRotation() );
    }

    public Rotation2d getAzimuthRotation() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
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