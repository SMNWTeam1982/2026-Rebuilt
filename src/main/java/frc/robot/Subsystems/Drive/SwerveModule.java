package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Measured.SwerveModuleMeasurements;
import frc.robot.Constants.Tunables.SwerveModuleTunables;

/**
 * this is NOT its own subsystem, this is only an abstraction for the drive subsystem that manages
 * one module
 */
public final class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;
    private final CANcoder turnEncoder;
    private final RelativeEncoder driveEncoder;

    private final PIDController turnPIDController;

    /** see the wpilib docs on Feedforward */
    private final SimpleMotorFeedforward driveFeedforward;

    private SwerveModuleState lastDesiredState;

    /** Constructs an instance of a SwerveModule with a drive motor, turn motor, and turn encoder. */
    public SwerveModule(int driveMotorCANID, int turnMotorCANID, int encoderCANID) {
        driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);
        driveMotor.configure(
                SwerveModuleTunables.DRIVE_MOTOR_CONFIG,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        turnMotor = new SparkMax(turnMotorCANID, MotorType.kBrushless);
        turnMotor.configure(
                SwerveModuleTunables.TURN_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turnEncoder = new CANcoder(encoderCANID);
        driveEncoder = driveMotor.getEncoder();

        turnPIDController = new PIDController(
                SwerveModuleTunables.TURN_P, SwerveModuleTunables.TURN_I, SwerveModuleTunables.TURN_D);

        driveFeedforward = new SimpleMotorFeedforward(
                SwerveModuleMeasurements.DRIVE_STATIC_GAIN,
                SwerveModuleMeasurements.DRIVE_VELOCITY_GAIN_VOLT_SECONDS_PER_METER);

        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * takes the given state and optimizes it, then sets the motors to move towards the state
     *
     * <p>this has to be called every frame in order to update work
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        lastDesiredState = desiredState;

        Rotation2d encoderRotation =
                Rotation2d.fromRotations(turnEncoder.getPosition().getValueAsDouble());

        desiredState.optimize(encoderRotation);

        desiredState.cosineScale(encoderRotation);

        double driveOutput = driveFeedforward.calculate(desiredState.speedMetersPerSecond);

        // the final safety checks for the speed
        driveOutput = MathUtil.clamp(driveOutput, -12.0, 12.0);

        if (Math.abs(driveOutput) < 0.001) {
            driveOutput = 0.0;
        }

        double turnOutput = -turnPIDController.calculate(encoderRotation.getRadians(), desiredState.angle.getRadians());

        // clamp the turn output
        turnOutput = MathUtil.clamp(turnOutput, -1.0, 1.0);

        driveMotor.setVoltage(driveOutput);
        turnMotor.set(turnOutput);
    }

    /** halts both the turn motor and the drive motor */
    public void stop() {
        turnMotor.set(0);
        driveMotor.set(0);
    }

    /** a method that can directly run the motors for debuging reasons */
    public void runMotors(double driveAmount, double turnAmount) {
        driveMotor.set(driveAmount);
        turnMotor.set(turnAmount);
    }

    /**
     * Updates the Turn motor PID
     *
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Derivate coefficient
     */
    public void updateTurnPID(double p, double i, double d) {
        turnPIDController.setPID(p, i, d);
    }

    /**
     * @return the last input to setDesiredState()
     */
    public SwerveModuleState getLastDesiredState() {
        return lastDesiredState;
    }

    /**
     * @return the angle of the wheel and the velocity of the wheel
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity() * SwerveModuleMeasurements.RPM_TO_MPS_MULTIPLIER,
                Rotation2d.fromRotations(turnEncoder.getPosition().getValueAsDouble()));
    }

    /**
     * @return the output current of the drive motor in amps,
     *     <p>this is data from the motor controller NOT the PDP
     */
    public double getDriveMotorOutputCurrent() {
        return driveMotor.getOutputCurrent();
    }

    /**
     * @return the angle of the wheel and the distance traveled by the wheel
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition() * SwerveModuleMeasurements.POSITION_TO_METERS_MULTIPLIER,
                Rotation2d.fromRotations(turnEncoder.getPosition().getValueAsDouble()));
    }

    public void logModuleData(String moduleName){
        Logger.recordOutput("DriveBase/" + moduleName + "/state", getState());
        Logger.recordOutput("DriveBase/" + moduleName + "/driveCurrent", getDriveMotorOutputCurrent());
    }
}
