package frc.lib;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;

/*
How to use:

import frc.lib.Telemetry;

Telemetry.setValue("bestTeam", "5411"); // "5411" will be assigned to the table key "bestTeam"
Telemetry.getValue("bestTeam", "0"); // "0" is the default value if the entry does not exist

*** be sure to use the style guide to name your keys ***
*/

public class Telemetry {
    /**
     * Fetched the default NetworkTable and works within the "telemetry" section of the table
     * @return the NetworkTable object cotaining all telemetry data
     */
    private static NetworkTable getTelemetryTable () {
        return NetworkTableInstance.getDefault().getTable("telemetry");
    }

    /** 
     * Set a telemetry value on the NetworkTable
     * @param key - the location to assign the value
     * @param value - the value to assign to the key
     */
    public static void setValue ( String key, Object value ) {
        getTelemetryTable().getEntry(key).setValue(value);
    }

    /** 
     * Set a telemetry value on the NetworkTable
     * @param key - the location to assign the value
     * @param value - the value to assign to the key
     */
    public static void setValue ( String key, String[] value ) {
        getTelemetryTable().getEntry(key).setStringArray(value);
    }

    /**
     * Get a telemetry value from the NetworkTable
     * @param key - the location to fetch the data
     * @param defaultValue - the default value to return if the data is absent
     * @return the data at the given key or the default value if there is no data
     */
    public static double getValue ( String key, double defaultValue ) {
        return getTelemetryTable().getEntry(key).getDouble(defaultValue);
    }

    /**
     * Get a telemetry value from the NetworkTable
     * @param key - the location to fetch the data
     * @param defaultValue - the default value to return if the data is absent
     * @return the data at the given key or the default value if there is no data
     */
    public static String getValue ( String key, String defaultValue ) {
        return getTelemetryTable().getEntry(key).getString(defaultValue);
    }

    /**
     * Get a telemetry value from the NetworkTable
     * @param key - the location to fetch the data
     * @param defaultValue - the default value to return if the data is absent
     * @return the data at the given key or the default value if there is no data
     */
    public static boolean getValue ( String key, boolean defaultValue ) {
        return getTelemetryTable().getEntry(key).getBoolean(defaultValue);
    }

    public static void flushTable () {
        NetworkTableInstance.getDefault().flush();
    }

    public static void armTelemetry(CANSparkMax motor, DutyCycleEncoder encoder, double offset, String key) {
        Telemetry.setValue("Arm/"+key+"/setpoint", motor.get());
        Telemetry.setValue("Arm/"+key+"/temperature", motor.getMotorTemperature());
        Telemetry.setValue("Arm/"+key+"/outputVoltage", motor.getAppliedOutput());
        Telemetry.setValue("Arm/"+key+"/statorCurrent", motor.getOutputCurrent());
        Telemetry.setValue("Arm/"+key+"/actualPosition", encoder.getAbsolutePosition() * 360);
        Telemetry.setValue("Arm/"+key+"/actualPositionOffset", encoder.getAbsolutePosition() * 360 - offset);
    }

    public static void intakeMotorTelemetry(String key, CANSparkMax motor) {
        Telemetry.setValue("Pincher/"+key+"/setpoint", motor.get());
        Telemetry.setValue("Pincher/"+key+"/temperature", motor.getMotorTemperature());
        Telemetry.setValue("Pincher/"+key+"/outputVoltage", motor.getAppliedOutput());
        Telemetry.setValue("Pincher/"+key+"/statorcurrent", motor.getOutputCurrent());
      }
}