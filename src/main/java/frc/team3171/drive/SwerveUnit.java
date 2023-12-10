package frc.team3171.drive;

// Java Imports
import java.util.function.DoubleSupplier;
import java.util.concurrent.ConcurrentLinkedQueue;

// FRC Imports
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// CTRE Imports
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

// REV Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.HelperFunctions;
import frc.team3171.drive.SwerveUnitConfig.MOTOR_TYPE;
import frc.team3171.sensors.ThreadedPIDController;

import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * @author Mark Ebert
 */
public class SwerveUnit implements DoubleSupplier, RobotProperties {

    // Motor Controllers
    private final CANSparkMax driveMotor, slewMotor;

    // CANCoder
    private final CANcoder absoluteEncoder;

    // PID Controller
    private final ThreadedPIDController slewPIDController;
    public final ConcurrentLinkedQueue<String> slewPIDData = new ConcurrentLinkedQueue<String>();

    // Global Variables
    private double startingAngle;

    /**
     * Constructor
     * 
     * @param driveInverted
     *            The config settings for the swerve unit.
     */
    public SwerveUnit(final SwerveUnitConfig swerveUnitConfig) {
        // Init the slew motor
        slewMotor = new CANSparkMax(swerveUnitConfig.getSLEW_MOTOR_CAN_ID(), MotorType.kBrushless);
        ((CANSparkMax) slewMotor).setIdleMode(IdleMode.kBrake);
        slewMotor.setInverted(swerveUnitConfig.isSLEW_MOTOR_INVERTED());

        // Init the drive motor

        driveMotor = new CANSparkMax(swerveUnitConfig.getDRIVE_MOTOR_CAN_ID(), MotorType.kBrushless);
        ((CANSparkMax) driveMotor).setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(swerveUnitConfig.isDRIVE_MOTOR_INVERTED());

        // Init the absolute position encoder used for the slew angle
        switch (swerveUnitConfig.getABSOLUTE_ENCODER_TYPE()) {
            case CTRE:
                absoluteEncoder = new CANcoder(swerveUnitConfig.getABSOLUTE_ENCODER_CAN_ID(),
                        swerveUnitConfig.getCANBUS());
                CANcoderConfiguration absoluteEncoderConfiguration = new CANcoderConfiguration();
                absoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
                absoluteEncoder.getConfigurator().apply(absoluteEncoderConfiguration);
                break;
            default:
                // Assumes the encoder is wired into the slew motor spark max
                absoluteEncoder = null;
                break;
        }

        // Init the gyro PID controller
        slewPIDController = new ThreadedPIDController(this::getAsDouble, SLEW_KP, SLEW_KI, SLEW_KD, SLEW_PID_MIN, SLEW_PID_MAX, true);
        slewPIDController.start(20, true, slewPIDData);

        // Init the global variables
        startingAngle = 0;
    }

    /**
     * Sets the drive motor to the desired speed.
     * 
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the drive motor to.
     */
    public void setDriveSpeed(final double speed) {
        /*
         * Sets the speed of the master TalonFX, and therefore it's followers, to the
         * given value
         */
        driveMotor.set(speed);
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     * 
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getDriveSpeed() {
        return driveMotor.get();
    }

    /**
     * Sets whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param inverted
     *            Whether or not the direction of the drive motor need to be
     *            inverted.
     */
    public void setDriveInverted(final boolean inverted) {
        /*
         * Sets whether or not the direction of the master TalonFX, and therefore it's
         * followers, need to be inverted
         */
        driveMotor.setInverted(inverted);
    }

    /**
     * Gets whether or not the direction of the drive motor is inverted.
     * 
     * @return True, if the drive motor is inverted, false otherwise.
     */
    public boolean getDriveInverted() {
        /*
         * Gets whether or not the direction of the master TalonFX, and therefore it's
         * followers, are inverted
         */
        return driveMotor.getInverted();
    }

    /**
     * Sets the slew motor to the desired angle.
     * 
     * @param angle
     *            The angle, from -180.0 to 180.0, to set the slew motor to.
     */
    public void updateSlewAngle(final double angle) {
        // Update the target angle of slew motor PID controller
        slewPIDController.updateSensorLockValueWithoutReset(angle);

        // Update Slew Motor Speed
        slewMotor.set(slewPIDController.getPIDValue());
    }

    /**
     * Updates the slew motors speed from the pid controller using the last updated
     * target angle.
     */
    public void updateSlewAngle() {
        // Update Slew Motor Speed
        slewMotor.set(slewPIDController.getPIDValue());
    }

    /**
     * Sets the slew motor to the desired speed.
     * 
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the slew motor to.
     */
    public void setSlewSpeed(final double speed) {
        slewMotor.set(speed);
    }

    /**
     * Returns the raw value of the {@link MotorController} integrated encoder.
     * 
     * @return The raw value from the {@link MotorController} integrated encoder.
     */
    public double getIntegratedEncoderValue() {
        return ((CANSparkMax) driveMotor).getEncoder().getPosition();
    }

    /**
     * Returns the velocity of the {@link MotorController} integrated encoder. If
     * the drive motor type is of
     * {@link MOTOR_TYPE.CTRE}, then the encoder 2048 ticks per revolution and the
     * return units of the velocity is in ticks
     * per 100ms. If the drive motor is of {@link MOTOR_TYPE.REV} then it returns
     * the RPM of the motor.
     * 
     * @return The velocity, in ticks per 100ms or RPM of the
     *         {@link MotorController} integrated encoder.
     */
    public double getIntegratedEncoderVelocity() {
        return ((CANSparkMax) driveMotor).getEncoder().getVelocity();
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     * 
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getSlewAngle() {

        // TODO Verify how to return the vlaue of the rev through bore encoder through
        // the spark max used for the slew motor and
        // its returned ranges.
        final double mappedEncoderAngle = HelperFunctions
                .Map(((CANSparkMax) slewMotor).getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition(), 0, 1, 0, 360);
        return Normalize_Gryo_Value(mappedEncoderAngle - startingAngle);
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     * 
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getSlewTargetAngle() {
        return slewPIDController.getSensorLockValue();
    }

    /**
     * Returns the current velocity of the {@link CANCoder}.
     * 
     * @return The velocity, in degrees per second.
     */
    public double getSlewVelocity() {
        return absoluteEncoder.getVelocity().getValueAsDouble();
    }

    public void enable() {
        slewPIDController.enablePID();
    }

    /**
     * Disables the drive and slew {@link TalonFX} motors.
     */
    public void disable() {
        driveMotor.disable();
        slewMotor.disable();
        slewPIDController.disablePID();
    }

    @Override
    public double getAsDouble() {
        return getSlewAngle();
    }

    public void zeroModule(final double slewOffset) {
        final double mappedEncoderAngle = HelperFunctions
                .Map(((CANSparkMax) slewMotor).getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition(), 0, 1, 0, 360);
        startingAngle = Normalize_Gryo_Value(mappedEncoderAngle - slewOffset);
    }

    public void zeroModule() {
        zeroModule(0);
    }

    public double getSlewOffset() {
        return startingAngle;
    }

    public void setSlewOffset(final double slewOffset) {
        startingAngle = slewOffset;
    }

}