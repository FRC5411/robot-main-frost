// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CAN.PIGEON_ID;
import frc.lib.Telemetry;

public class Pigeon extends SubsystemBase {
  private Pigeon2 pigeon;

  private double yaw = 0;
  private double pitch = 0;
  private double roll = 0;
  private double[] ypr = new double[3];

    /** Creates a new ExampleSubsystem. */
  public Pigeon () {
    pigeon = new Pigeon2(PIGEON_ID);
  }

  public Pigeon2 getPigeon () {
      return pigeon;
  }

  public double getYaw() {
    return yaw;
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

  @Override
  public void periodic() {
    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    pigeon.getYawPitchRoll(ypr);

    Telemetry.setValue("drivetrain/gyro/temperature", pigeon.getTemp());
    Telemetry.setValue("drivetrain/gyro/yaw", yaw);
    Telemetry.setValue("drivetrain/gyro/roll", roll);
    Telemetry.setValue("drivetrain/gyro/pitch", pitch);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}