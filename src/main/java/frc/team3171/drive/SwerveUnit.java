package frc.team3171.drive;

// Java Imports
import java.util.function.DoubleSupplier;

// FRC Imports
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// CTRE Imports
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

// REV Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.drive.SwerveUnitConfig.MOTOR_TYPE;
import frc.team3171.sensors.ThreadedPIDController;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * @author Mark Ebert
 */
public class SwerveUnit implements DoubleSupplier, RobotProperties {

    // Swerve Config
    private final SwerveUnitConfig SWERVE_UNIT_CONFIG;

    // Motor Controllers
    private final MotorController driveMotor, slewMotor;

    // CANCoder
    private final CANCoder absoluteEncoder;

    // PID Controller
    private final ThreadedPIDController slewPIDController;

    // Global Variables
    private double startingAngle;

    /**
     * Constructor
     * 
     * @param driveInverted
     *            The config settings for the swerve unit.
     */
    public SwerveUnit(final SwerveUnitConfig swerveUnitConfig) {
        this.SWERVE_UNIT_CONFIG = swerveUnitConfig;

        // Init the slew motor
        switch (swerveUnitConfig.getSLEW_MOTOR_TYPE()) {
            case CTRE:
                slewMotor = new FRCTalonFX(swerveUnitConfig.getSLEW_MOTOR_CAN_ID(), swerveUnitConfig.getCANBUS());
                break;
            default:
                slewMotor = new CANSparkMax(swerveUnitConfig.getSLEW_MOTOR_CAN_ID(), MotorType.kBrushless);
                ((CANSparkMax) slewMotor).setIdleMode(IdleMode.kBrake);
                break;
        }
        slewMotor.setInverted(swerveUnitConfig.isSLEW_MOTOR_INVERTED());

        // Init the drive motor
        switch (swerveUnitConfig.getDRIVE_MOTOR_TYPE()) {
            case CTRE:
                driveMotor = new FRCTalonFX(swerveUnitConfig.getDRIVE_MOTOR_CAN_ID(), swerveUnitConfig.getCANBUS());

                break;
            default:
                driveMotor = new CANSparkMax(swerveUnitConfig.getDRIVE_MOTOR_CAN_ID(), MotorType.kBrushless);
                ((CANSparkMax) driveMotor).setIdleMode(IdleMode.kBrake);
                break;
        }
        driveMotor.setInverted(swerveUnitConfig.isDRIVE_MOTOR_INVERTED());

        // Init the absolute position encoder used for the slew angle
        switch (swerveUnitConfig.getABSOLUTE_ENCODER_TYPE()) {
            case CTRE:
                absoluteEncoder = new CANCoder(swerveUnitConfig.getABSOLUTE_ENCODER_CAN_ID(), swerveUnitConfig.getCANBUS());
                absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
                break;
            default:
                // Assumes the encoder is wired into the slew motor spark max
                absoluteEncoder = null;
                break;
        }

        // Init the gyro PID controller
        slewPIDController = new ThreadedPIDController(this, SLEW_KP, SLEW_KI, SLEW_KD, SLEW_PID_MIN, SLEW_PID_MAX, true);
        slewPIDController.start(true);

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
     * Updates the slew motors speed from the pid controller using the last updated target angle.
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
        switch (SWERVE_UNIT_CONFIG.getDRIVE_MOTOR_TYPE()) {
            case CTRE:
                return ((FRCTalonFX) driveMotor).getSelectedSensorPosition();
            default:
                return ((CANSparkMax) driveMotor).getEncoder().getPosition();
        }
    }

    /**
     * Returns the velocity of the {@link MotorController} integrated encoder. If the drive motor type is of
     * {@link MOTOR_TYPE.CTRE}, then the encoder 2048 ticks per revolution and the return units of the velocity is in ticks
     * per 100ms. If the drive motor is of {@link MOTOR_TYPE.REV} then it returns the RPM of the motor.
     * 
     * @return The velocity, in ticks per 100ms or RPM of the {@link MotorController} integrated encoder.
     */
    public double getIntegratedEncoderVelocity() {
        switch (SWERVE_UNIT_CONFIG.getDRIVE_MOTOR_TYPE()) {
            case CTRE:
                return ((FRCTalonFX) driveMotor).getSelectedSensorVelocity();
            default:
                return ((CANSparkMax) driveMotor).getEncoder().getVelocity();
        }
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     * 
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getSlewAngle() {
        switch (SWERVE_UNIT_CONFIG.getABSOLUTE_ENCODER_TYPE()) {
            case CTRE:
                return Normalize_Gryo_Value(absoluteEncoder.getAbsolutePosition() - startingAngle);
            default:
                // TODO Verify how to return the vlaue of the rev through bore encoder through the spark max used for the slew motor and
                // its returned ranges.
                return Normalize_Gryo_Value(((CANSparkMax) slewMotor).getAlternateEncoder(2048).getPosition() - startingAngle);
        }
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
        return absoluteEncoder.getVelocity();
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

    public void zeroModule() {
        switch (SWERVE_UNIT_CONFIG.getABSOLUTE_ENCODER_TYPE()) {
            case CTRE:
                startingAngle = absoluteEncoder.getAbsolutePosition();
                break;
            default:
                // TODO Verify how to return the vlaue of the rev through bore encoder through the spark max used for the slew motor and
                // its returned ranges.
                startingAngle = ((CANSparkMax) slewMotor).getAlternateEncoder(2048).getPosition();
                break;
        }
    }

    public double getSlewOffset() {
        return startingAngle;
    }

    public void setSlewOffset(final double slewOffset) {
        startingAngle = slewOffset;
    }

}