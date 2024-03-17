package frc.team3171.drive;

// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;

// FRC Imports
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

// CTRE Imports
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

// REV Imports
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.controllers.ThreadedPIDController;
import frc.team3171.protos.SlewDrive.SlewUnitConfiguration;
import static frc.team3171.HelperFunctions.Map;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * @author Mark Ebert
 */
public class SwerveUnit implements RobotProperties {

    private final SlewUnitConfiguration slewUnitConfiguration;

    // Motor Controllers
    private final MotorController driveMotor, slewMotor;

    // Absolute Encoder
    private final Object absoluteEncoder;

    // Relative Encoder
    private final RelativeEncoder relativeDriveEncoder, relativeSlewEncoder;

    // PID Controller
    private final ThreadedPIDController slewPIDController;
    private final ConcurrentLinkedQueue<String> slewPIDData;

    // Global Variables
    private double startingAngle;

    /**
     * Constructor
     * 
     * @param driveInverted The config settings for the swerve unit.
     */
    public SwerveUnit(final SlewUnitConfiguration slewUnitConfiguration) {
        this.slewUnitConfiguration = slewUnitConfiguration;

        // Init the drive motor
        switch (slewUnitConfiguration.getDriveMotorType()) {
        case CTRE_FALCON:
            driveMotor = new TalonFX(slewUnitConfiguration.getDriveMotorID(), slewUnitConfiguration.getCanbus());
            ((TalonFX) driveMotor).setInverted(slewUnitConfiguration.getDriveMotorInverted());
            ((TalonFX) driveMotor).setNeutralMode(NeutralModeValue.Brake);
            relativeDriveEncoder = null;
            break;
        case REV_SPARK_FLEX:
            driveMotor = new CANSparkFlex(slewUnitConfiguration.getDriveMotorID(), MotorType.kBrushless);
            ((CANSparkFlex) driveMotor).restoreFactoryDefaults();
            ((CANSparkFlex) driveMotor).setInverted(slewUnitConfiguration.getDriveMotorInverted());
            ((CANSparkFlex) driveMotor).setIdleMode(IdleMode.kBrake);
            ((CANSparkFlex) driveMotor).burnFlash();
            relativeDriveEncoder = ((CANSparkFlex) driveMotor).getEncoder();
            break;
        default:
            driveMotor = new CANSparkMax(slewUnitConfiguration.getDriveMotorID(), MotorType.kBrushless);
            ((CANSparkMax) driveMotor).restoreFactoryDefaults();
            ((CANSparkMax) driveMotor).setInverted(slewUnitConfiguration.getDriveMotorInverted());
            ((CANSparkMax) driveMotor).setIdleMode(IdleMode.kBrake);
            ((CANSparkMax) driveMotor).burnFlash();
            relativeDriveEncoder = ((CANSparkMax) driveMotor).getEncoder();
            break;
        }

        // Init the slew motor
        switch (slewUnitConfiguration.getSlewMotorType()) {
        case CTRE_FALCON:
            slewMotor = new TalonFX(slewUnitConfiguration.getSlewMotorID(), slewUnitConfiguration.getCanbus());
            ((TalonFX) slewMotor).setInverted(slewUnitConfiguration.getSlewMotorInverted());
            ((TalonFX) slewMotor).setNeutralMode(NeutralModeValue.Brake);
            relativeSlewEncoder = null;
            break;
        case REV_SPARK_FLEX:
            slewMotor = new CANSparkFlex(slewUnitConfiguration.getSlewMotorID(), MotorType.kBrushless);
            ((CANSparkFlex) slewMotor).restoreFactoryDefaults();
            ((CANSparkFlex) slewMotor).setInverted(slewUnitConfiguration.getSlewMotorInverted());
            ((CANSparkFlex) slewMotor).setIdleMode(IdleMode.kBrake);
            ((CANSparkFlex) slewMotor).burnFlash();
            relativeSlewEncoder = ((CANSparkFlex) slewMotor).getEncoder();
            break;
        default:
            slewMotor = new CANSparkMax(slewUnitConfiguration.getSlewMotorID(), MotorType.kBrushless);
            ((CANSparkMax) slewMotor).restoreFactoryDefaults();
            ((CANSparkMax) slewMotor).setInverted(slewUnitConfiguration.getSlewMotorInverted());
            ((CANSparkMax) slewMotor).setIdleMode(IdleMode.kBrake);
            ((CANSparkMax) slewMotor).burnFlash();
            relativeSlewEncoder = ((CANSparkMax) slewMotor).getEncoder();
            break;
        }

        // Init the absolute position encoder used for the slew angle
        switch (slewUnitConfiguration.getAbsoluteEncoderType()) {
        case CTRE_CANCODER:
            absoluteEncoder = new CANcoder(slewUnitConfiguration.getAbsoluteEncoderID(), slewUnitConfiguration.getCanbus());
            CANcoderConfiguration absoluteEncoderConfiguration = new CANcoderConfiguration();
            absoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            absoluteEncoderConfiguration.MagnetSensor.SensorDirection = slewUnitConfiguration.getAbsoluteEncoderInverted() ? SensorDirectionValue.CounterClockwise_Positive
                    : SensorDirectionValue.Clockwise_Positive;
            ((CANcoder) absoluteEncoder).getConfigurator().apply(absoluteEncoderConfiguration);
            break;
        default:
            // Assumes duty cycle absolute encoder because the REV connector on the SparkMax SUCK
            absoluteEncoder = new DutyCycleEncoder(slewUnitConfiguration.getAbsoluteEncoderID());
            break;
        }

        // Init the queue for pid data, if enabled
        slewPIDData = slewUnitConfiguration.getLogPIDData() ? new ConcurrentLinkedQueue<String>() : null;

        // Init the gyro PID controller
        slewPIDController = new ThreadedPIDController(this::getSlewAngle, SLEW_KP, SLEW_KI, SLEW_KD, SLEW_PID_MIN, SLEW_PID_MAX, false);
        slewPIDController.setMinSensorValue(-180);
        slewPIDController.setMaxSensorValue(180);
        slewPIDController.start(20, true, slewPIDData);

        // Init the global variables
        startingAngle = 0;
    }

    /**
     * Sets the drive motor to the desired speed.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the drive motor to.
     */
    public void setDriveSpeed(final double speed) {
        /*
         * Sets the speed of the master TalonFX, and therefore it's followers, to the given value
         */
        switch (slewUnitConfiguration.getDriveMotorType()) {
        case CTRE_FALCON:
            ((TalonFX) driveMotor).set(speed);
            break;
        case REV_SPARK_FLEX:
            ((CANSparkFlex) driveMotor).set(speed);
            break;
        default:
            ((CANSparkMax) driveMotor).set(speed);
            break;
        }
    }

    /**
     * Returns the current drive motor speed.
     * 
     * @return The current drive motor speed, from -1.0 to 1.0.
     */
    public double getDriveSpeed() {
        switch (slewUnitConfiguration.getDriveMotorType()) {
        case CTRE_FALCON:
            return ((TalonFX) driveMotor).get();
        case REV_SPARK_FLEX:
            return ((CANSparkFlex) driveMotor).get();
        default:
            return ((CANSparkMax) driveMotor).get();
        }
    }

    /**
     * Sets whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param inverted Whether or not the direction of the drive motor need to be inverted.
     */
    public void setDriveInverted(final boolean inverted) {
        /*
         * Sets whether or not the direction of the master TalonFX, and therefore it's followers, need to be inverted
         */
        switch (slewUnitConfiguration.getDriveMotorType()) {
        case CTRE_FALCON:
            ((TalonFX) driveMotor).setInverted(inverted);
            break;
        case REV_SPARK_FLEX:
            ((CANSparkFlex) driveMotor).setInverted(inverted);
            break;
        default:
            ((CANSparkMax) driveMotor).setInverted(inverted);
            break;
        }
    }

    /**
     * Gets whether or not the direction of the drive motor is inverted.
     * 
     * @return True, if the drive motor is inverted, false otherwise.
     */
    public boolean getDriveInverted() {
        /*
         * Gets whether or not the direction of the master TalonFX, and therefore it's followers, are inverted
         */
        switch (slewUnitConfiguration.getDriveMotorType()) {
        case CTRE_FALCON:
            return ((TalonFX) driveMotor).getInverted();
        case REV_SPARK_FLEX:
            return ((CANSparkFlex) driveMotor).getInverted();
        default:
            return ((CANSparkMax) driveMotor).getInverted();
        }
    }

    /**
     * Updates the slew motors speed from the pid controller using the last updated target angle.
     */
    public void updateSlewAngle() {
        // Update Slew Motor Speed
        switch (slewUnitConfiguration.getSlewMotorType()) {
        case CTRE_FALCON:
            ((TalonFX) slewMotor).set(slewPIDController.getPIDValue());
            break;
        case REV_SPARK_FLEX:
            ((CANSparkFlex) slewMotor).set(slewPIDController.getPIDValue());
            break;
        default:
            ((CANSparkMax) slewMotor).set(slewPIDController.getPIDValue());
            break;
        }
    }

    /**
     * Sets the slew motor to the desired angle.
     * 
     * @param angle The angle, from -180.0 to 180.0, to set the slew motor to.
     */
    public void updateSlewAngle(final double angle) {
        // Update the target angle of slew motor PID controller
        slewPIDController.updateSensorLockValueWithoutReset(angle);
        updateSlewAngle();
    }

    /**
     * Sets the slew motor to the desired speed.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the slew motor to.
     */
    public void setSlewSpeed(final double speed) {
        switch (slewUnitConfiguration.getSlewMotorType()) {
        case CTRE_FALCON:
            ((TalonFX) slewMotor).set(speed);
            break;
        case REV_SPARK_FLEX:
            ((CANSparkFlex) slewMotor).set(speed);
            break;
        default:
            ((CANSparkMax) slewMotor).set(speed);
            break;
        }
    }

    /**
     * Returns the raw value of the {@link MotorController} integrated encoder.
     * 
     * @return The raw value from the {@link MotorController} integrated encoder.
     */
    public double getIntegratedEncoderValue() {
        switch (slewUnitConfiguration.getDriveMotorType()) {
        case CTRE_FALCON:
            return ((TalonFX) driveMotor).getPosition().getValueAsDouble();
        default:
            return relativeDriveEncoder != null ? relativeDriveEncoder.getPosition() : 0;
        }
    }

    /**
     * Returns the velocity of the {@link MotorController} integrated encoder. If the drive motor type is of {@link MOTOR_TYPE.CTRE}, then the encoder 2048 ticks per revolution and the return units of
     * the velocity is in ticks per 100ms. If the drive motor is of {@link MOTOR_TYPE.REV} then it returns the RPM of the motor.
     * 
     * @return The velocity, in ticks per 100ms or RPM of the {@link MotorController} integrated encoder.
     */
    public double getIntegratedEncoderVelocity() {
        switch (slewUnitConfiguration.getDriveMotorType()) {
        case CTRE_FALCON:
            return ((TalonFX) driveMotor).getVelocity().getValueAsDouble();
        default:
            return relativeDriveEncoder != null ? relativeDriveEncoder.getVelocity() : 0;
        }
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     * 
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getSlewAngle() {
        final double mappedEncoderAngle;

        // Get the absolute encoder value based on encoder type
        switch (slewUnitConfiguration.getAbsoluteEncoderType()) {
        case CTRE_CANCODER:
            mappedEncoderAngle = Map(((CANcoder) absoluteEncoder).getAbsolutePosition().getValueAsDouble(), 0, 1, 0, 360);
            break;
        default:
            // Assumes the encoder is wired into the slew motor spark max
            mappedEncoderAngle = Map(((DutyCycleEncoder) absoluteEncoder).getAbsolutePosition(), 0, 1, 0, 360);
            break;
        }
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
        final double slewVelocity;
        // Get the absolute encoder value based on encoder type
        switch (slewUnitConfiguration.getAbsoluteEncoderType()) {
        case CTRE_CANCODER:
            slewVelocity = ((CANcoder) absoluteEncoder).getVelocity().getValueAsDouble();
            break;
        default:
            switch (slewUnitConfiguration.getSlewMotorType()) {
            case CTRE_FALCON:
                // If theres no cancoder and not a REV motor then get the value from the TalonFX integrated encoder
                slewVelocity = ((TalonFX) slewMotor).getVelocity().getValueAsDouble();
                break;
            default:
                // Assumes the encoder is wired into the slew motor spark max
                slewVelocity = relativeSlewEncoder != null ? relativeSlewEncoder.getVelocity() : 0;
                break;
            }
            break;
        }
        return slewVelocity;
    }

    /**
     * Enables the slew unit
     */
    public void enable() {
        slewPIDController.enablePID();
    }

    /**
     * Disables the drive and slew {@link TalonFX} motors.
     */
    public void disable() {
        switch (slewUnitConfiguration.getDriveMotorType()) {
        case CTRE_FALCON:
            ((TalonFX) driveMotor).disable();
            break;
        case REV_SPARK_FLEX:
            ((CANSparkFlex) driveMotor).disable();
            break;
        default:
            ((CANSparkMax) driveMotor).disable();
            break;
        }
        switch (slewUnitConfiguration.getSlewMotorType()) {
        case CTRE_FALCON:
            ((TalonFX) slewMotor).disable();
            break;
        case REV_SPARK_FLEX:
            ((CANSparkFlex) slewMotor).disable();
            break;
        default:
            ((CANSparkMax) slewMotor).disable();
            break;
        }
        slewPIDController.disablePID();
    }

    /**
     * 
     * @param slewOffset
     */
    public void zeroModule(final double slewOffset) {
        final double mappedEncoderAngle;

        // Get the absolute encoder value based on encoder type
        switch (slewUnitConfiguration.getAbsoluteEncoderType()) {
        case CTRE_CANCODER:
            mappedEncoderAngle = Map(((CANcoder) absoluteEncoder).getAbsolutePosition().getValueAsDouble(), 0, 1, 0, 360);
            break;
        default:
            // Assumes the encoder is wired into the slew motor spark max
            mappedEncoderAngle = Map(((DutyCycleEncoder) absoluteEncoder).getAbsolutePosition(), 0, 1, 0, 360);
            break;
        }
        startingAngle = Normalize_Gryo_Value(mappedEncoderAngle - slewOffset);
    }

    /**
     * 
     */
    public void zeroModule() {
        zeroModule(0);
    }

    /**
     * 
     * @return
     */
    public double getSlewOffset() {
        return startingAngle;
    }

    /**
     * 
     * @param slewOffset
     */
    public void setSlewOffset(final double slewOffset) {
        startingAngle = slewOffset;
    }

    /**
     * 
     * @return
     */
    public ConcurrentLinkedQueue<String> getSlewPIDData() {
        return slewPIDData;
    }

}