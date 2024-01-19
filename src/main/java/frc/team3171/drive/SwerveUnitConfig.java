package frc.team3171.drive;

/**
 * @author Mark Ebert
 */
public class SwerveUnitConfig {

    public enum MOTOR_TYPE {
        CTRE, REV
    }

    public enum ENCODER_TYPE {
        CTRE, REV
    }

    private final MOTOR_TYPE DRIVE_MOTOR_TYPE, SLEW_MOTOR_TYPE;
    private final ENCODER_TYPE ABSOLUTE_ENCODER_TYPE;
    private final int SLEW_MOTOR_CAN_ID, DRIVE_MOTOR_CAN_ID, ABSOLUTE_ENCODER_CAN_ID;
    private final boolean SLEW_MOTOR_INVERTED, DRIVE_MOTOR_INVERTED, ABSOLUTE_ENCODER_INVERTED;
    private final String CANBUS;
    private final boolean LOG_PID_DATA;

    /**
     * Constructor
     * 
     * @param SLEW_MOTOR_TYPE
     *            The motor type of the {@link MotorController} used to control the slew motor.
     * @param SLEW_MOTOR_CAN_ID
     *            The CAN ID of the {@link MotorController} used to control the slew motor.
     * @param SLEW_MOTOR_INVERTED
     *            Whether or not the direction of the slew motor needs to be inverted.
     * 
     * @param DRIVE_MOTOR_TYPE
     *            The motor type of the {@link MotorController} used to control the drive motor.
     * @param DRIVE_MOTOR_CAN_ID
     *            The CAN ID of the {@link MotorController} used to control the drive motor.
     * @param DRIVE_MOTOR_INVERTED
     *            Whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param ABSOLUTE_ENCODER_TYPE
     *            The encoder type used to tell the positiion of the wheel.
     * @param ABSOLUTE_ENCODER_CAN_ID
     *            The CAN ID of the encoder used to tell the positiion of the wheel.
     * @param ABSOLUTE_ENCODER_INVERTED
     *            Whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param CANBUS
     *            The {@link String} of the CAN bus that the devices are located on. All three devices of the unit should be
     *            on the same bus.
     * 
     * @param LOG_PID_DATA
     *            Whether or not to log the PID values from this swerve unit.
     */
    public SwerveUnitConfig(
            final MOTOR_TYPE SLEW_MOTOR_TYPE, final int SLEW_MOTOR_CAN_ID, final boolean SLEW_MOTOR_INVERTED,
            final MOTOR_TYPE DRIVE_MOTOR_TYPE, final int DRIVE_MOTOR_CAN_ID, final boolean DRIVE_MOTOR_INVERTED,
            final ENCODER_TYPE ABSOLUTE_ENCODER_TYPE, final int ABSOLUTE_ENCODER_CAN_ID, final boolean ABSOLUTE_ENCODER_INVERTED, final String CANBUS,
            final boolean LOG_PID_DATA) {
        this.SLEW_MOTOR_TYPE = SLEW_MOTOR_TYPE;
        this.SLEW_MOTOR_CAN_ID = SLEW_MOTOR_CAN_ID;
        this.SLEW_MOTOR_INVERTED = SLEW_MOTOR_INVERTED;

        this.DRIVE_MOTOR_TYPE = DRIVE_MOTOR_TYPE;
        this.DRIVE_MOTOR_CAN_ID = DRIVE_MOTOR_CAN_ID;
        this.DRIVE_MOTOR_INVERTED = DRIVE_MOTOR_INVERTED;

        this.ABSOLUTE_ENCODER_TYPE = ABSOLUTE_ENCODER_TYPE;
        this.ABSOLUTE_ENCODER_CAN_ID = ABSOLUTE_ENCODER_CAN_ID;
        this.ABSOLUTE_ENCODER_INVERTED = ABSOLUTE_ENCODER_INVERTED;

        this.CANBUS = CANBUS;

        this.LOG_PID_DATA = LOG_PID_DATA;
    }

    /**
     * Constructor
     * 
     * @param SLEW_MOTOR_TYPE
     *            The motor type of the {@link MotorController} used to control the slew motor.
     * @param SLEW_MOTOR_CAN_ID
     *            The CAN ID of the {@link MotorController} used to control the slew motor.
     * @param SLEW_MOTOR_INVERTED
     *            Whether or not the direction of the slew motor needs to be inverted.
     * 
     * @param DRIVE_MOTOR_TYPE
     *            The motor type of the {@link MotorController} used to control the drive motor.
     * @param DRIVE_MOTOR_CAN_ID
     *            The CAN ID of the {@link MotorController} used to control the drive motor.
     * @param DRIVE_MOTOR_INVERTED
     *            Whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param ABSOLUTE_ENCODER_TYPE
     *            The encoder type used to tell the positiion of the wheel.
     * @param ABSOLUTE_ENCODER_CAN_ID
     *            The CAN ID of the encoder used to tell the positiion of the wheel.
     * @param ABSOLUTE_ENCODER_INVERTED
     *            Whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param CANBUS
     *            The {@link String} of the CAN bus that the devices are located on. All three devices of the unit should be
     *            on the same bus.
     */
    public SwerveUnitConfig(
            final MOTOR_TYPE SLEW_MOTOR_TYPE, final int SLEW_MOTOR_CAN_ID, final boolean SLEW_MOTOR_INVERTED,
            final MOTOR_TYPE DRIVE_MOTOR_TYPE, final int DRIVE_MOTOR_CAN_ID, final boolean DRIVE_MOTOR_INVERTED,
            final ENCODER_TYPE ABSOLUTE_ENCODER_TYPE, final int ABSOLUTE_ENCODER_CAN_ID, final boolean ABSOLUTE_ENCODER_INVERTED, final String CANBUS) {
        this(SLEW_MOTOR_TYPE, SLEW_MOTOR_CAN_ID, SLEW_MOTOR_INVERTED, DRIVE_MOTOR_TYPE, DRIVE_MOTOR_CAN_ID, DRIVE_MOTOR_INVERTED, ABSOLUTE_ENCODER_TYPE,
                ABSOLUTE_ENCODER_CAN_ID, ABSOLUTE_ENCODER_INVERTED, CANBUS, false);
    }

    /**
     * Constructor
     * 
     * @param SLEW_MOTOR_TYPE
     *            The motor type of the {@link MotorController} used to control the slew motor.
     * @param SLEW_MOTOR_CAN_ID
     *            The CAN ID of the {@link MotorController} used to control the slew motor.
     * @param SLEW_MOTOR_INVERTED
     *            Whether or not the direction of the slew motor needs to be inverted.
     * 
     * @param DRIVE_MOTOR_TYPE
     *            The motor type of the {@link MotorController} used to control the drive motor.
     * @param DRIVE_MOTOR_CAN_ID
     *            The CAN ID of the {@link MotorController} used to control the drive motor.
     * @param DRIVE_MOTOR_INVERTED
     *            Whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param ABSOLUTE_ENCODER_TYPE
     *            The encoder type used to tell the positiion of the wheel.
     * @param ABSOLUTE_ENCODER_CAN_ID
     *            The CAN ID of the encoder used to tell the positiion of the wheel.
     * @param ABSOLUTE_ENCODER_INVERTED
     *            Whether or not the direction of the drive motor needs to be inverted.
     */
    public SwerveUnitConfig(
            final MOTOR_TYPE SLEW_MOTOR_TYPE, final int SLEW_MOTOR_CAN_ID, final boolean SLEW_MOTOR_INVERTED,
            final MOTOR_TYPE DRIVE_MOTOR_TYPE, final int DRIVE_MOTOR_CAN_ID, final boolean DRIVE_MOTOR_INVERTED,
            final ENCODER_TYPE ABSOLUTE_ENCODER_TYPE, final int ABSOLUTE_ENCODER_CAN_ID, final boolean ABSOLUTE_ENCODER_INVERTED) {
        this(SLEW_MOTOR_TYPE, SLEW_MOTOR_CAN_ID, SLEW_MOTOR_INVERTED, DRIVE_MOTOR_TYPE, DRIVE_MOTOR_CAN_ID, DRIVE_MOTOR_INVERTED, ABSOLUTE_ENCODER_TYPE,
                ABSOLUTE_ENCODER_CAN_ID, ABSOLUTE_ENCODER_INVERTED, "", false);
    }

    /**
     * Constructor
     * 
     * @param SLEW_MOTOR_TYPE
     *            The motor type of the {@link MotorController} used to control the slew motor.
     * @param SLEW_MOTOR_CAN_ID
     *            The CAN ID of the {@link MotorController} used to control the slew motor.
     * @param SLEW_MOTOR_INVERTED
     *            Whether or not the direction of the slew motor needs to be inverted.
     * 
     * @param DRIVE_MOTOR_TYPE
     *            The motor type of the {@link MotorController} used to control the drive motor.
     * @param DRIVE_MOTOR_CAN_ID
     *            The CAN ID of the {@link MotorController} used to control the drive motor.
     * @param DRIVE_MOTOR_INVERTED
     *            Whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param ABSOLUTE_ENCODER_TYPE
     *            The encoder type used to tell the positiion of the wheel.
     * @param ABSOLUTE_ENCODER_CAN_ID
     *            The CAN ID of the encoder used to tell the positiion of the wheel.
     * @param ABSOLUTE_ENCODER_INVERTED
     *            Whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param LOG_PID_DATA
     *            Whether or not to log the PID values from this swerve unit.
     */
    public SwerveUnitConfig(
            final MOTOR_TYPE SLEW_MOTOR_TYPE, final int SLEW_MOTOR_CAN_ID, final boolean SLEW_MOTOR_INVERTED,
            final MOTOR_TYPE DRIVE_MOTOR_TYPE, final int DRIVE_MOTOR_CAN_ID, final boolean DRIVE_MOTOR_INVERTED,
            final ENCODER_TYPE ABSOLUTE_ENCODER_TYPE, final int ABSOLUTE_ENCODER_CAN_ID, final boolean ABSOLUTE_ENCODER_INVERTED, final boolean LOG_PID_DATA) {
        this(SLEW_MOTOR_TYPE, SLEW_MOTOR_CAN_ID, SLEW_MOTOR_INVERTED, DRIVE_MOTOR_TYPE, DRIVE_MOTOR_CAN_ID, DRIVE_MOTOR_INVERTED, ABSOLUTE_ENCODER_TYPE,
                ABSOLUTE_ENCODER_CAN_ID, ABSOLUTE_ENCODER_INVERTED, "", LOG_PID_DATA);
    }

    public MOTOR_TYPE getDRIVE_MOTOR_TYPE() {
        return DRIVE_MOTOR_TYPE;
    }

    public MOTOR_TYPE getSLEW_MOTOR_TYPE() {
        return SLEW_MOTOR_TYPE;
    }

    public ENCODER_TYPE getABSOLUTE_ENCODER_TYPE() {
        return ABSOLUTE_ENCODER_TYPE;
    }

    public int getSLEW_MOTOR_CAN_ID() {
        return SLEW_MOTOR_CAN_ID;
    }

    public int getDRIVE_MOTOR_CAN_ID() {
        return DRIVE_MOTOR_CAN_ID;
    }

    public int getABSOLUTE_ENCODER_CAN_ID() {
        return ABSOLUTE_ENCODER_CAN_ID;
    }

    public boolean isSLEW_MOTOR_INVERTED() {
        return SLEW_MOTOR_INVERTED;
    }

    public boolean isDRIVE_MOTOR_INVERTED() {
        return DRIVE_MOTOR_INVERTED;
    }

    public boolean isABSOLUTE_ENCODER_INVERTED() {
        return ABSOLUTE_ENCODER_INVERTED;
    }

    public String getCANBUS() {
        return CANBUS;
    }

    public boolean getLOG_PID_DATA() {
        return LOG_PID_DATA;
    }

}
