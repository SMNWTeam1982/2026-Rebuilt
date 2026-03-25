package frc.robot;

import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.Logger;

public final class SparkMaxHelper {
    /** logs the motor output, current, and temperature */
    public static void logMotorDetails(String subsystemName, String motorName, SparkMax motor) {
        String path = subsystemName + "/" + motorName + "/";

        Logger.recordOutput(path + "output", motor.getAppliedOutput());
        Logger.recordOutput(path + "current", motor.getOutputCurrent());
        Logger.recordOutput(path + "temperature", motor.getMotorTemperature());
    }
}
