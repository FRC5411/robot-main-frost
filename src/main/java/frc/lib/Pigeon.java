// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import static frc.robot.Constants.CAN.PIGEON_ID;
import edu.wpi.first.wpilibj.DriverStation;

public class Pigeon {
  private Pigeon2 pigeon;

  private double yaw = 0;
  private double pitch = 0;
  private double roll = 0;
  private double[] ypr = new double[3];

  public Pigeon () {
    pigeon = new Pigeon2(PIGEON_ID, "drivetrain");
    zeroYaw();
  }

  public Pigeon2 getPigeon () {
      return pigeon;
  }

  public double getYaw() {
    return yaw % 360;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }

  public Rotation2d getAllianceRotation2d() {
    return getRotation2d().plus( Rotation2d.fromDegrees(
      ( DriverStation.getAlliance().equals( DriverStation.Alliance.Red ) ) ? 180 : 0 ) );
  }

  public double getPitch () {
    return pitch;
  }

  public double getRoll () {
    return roll;
  }

  public double[] getYPR () {
    return ypr;
  }

  public void zeroYaw () {
    pigeon.setYaw(0);
  }

  public void periodic() {
    yaw = pigeon.getYaw() % 360;
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    pigeon.getYawPitchRoll(ypr);

    Telemetry.setValue("drivetrain/gyro/temperature", pigeon.getTemp());
    Telemetry.setValue("drivetrain/gyro/yaw", yaw);
    Telemetry.setValue("drivetrain/gyro/roll", roll);
    Telemetry.setValue("drivetrain/gyro/pitch", pitch);
  }
}