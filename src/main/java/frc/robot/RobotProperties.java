package frc.robot;

// Team 3171 Imports
import frc.team3171.drive.SwerveUnitConfig;
import frc.team3171.drive.SwerveUnitConfig.ENCODER_TYPE;
import frc.team3171.drive.SwerveUnitConfig.MOTOR_TYPE;
import frc.team3171.drive.SwerveUnitConfig.SwerveUnitConfigBuilder;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

        /** Debug Options **/
        public static final boolean DEBUG = true;
        public static final boolean SWERVE_DIRECTION_DEBUG = false;
        public static final String PID_LOG_ADDRESS = "10.31.71.201";

        /** Drive Variables **/
        public static final boolean FIELD_ORIENTED_SWERVE = false;
        public static final double JOYSTICK_DEADZONE = .08;
        public static final double MAX_DRIVE_SPEED = .5, MAX_ROTATION_SPEED = .6;
        public static final boolean PINWHEEL_ZERO_ORIENTATION = false;
        public static final boolean SWERVE_UNIT_ORIENTATION_OPTIMIZATION = true;

        /** Swerve Unit Configuration **/
        public static final SwerveUnitConfig lf_Unit_Config = new SwerveUnitConfigBuilder(MOTOR_TYPE.CTRE, 12, MOTOR_TYPE.CTRE, 13, ENCODER_TYPE.CTRE)
                        .setAbsoluteEncoderCANID(19).setCANBUS("canivore").setSlewMotorInverted(false).build();
        public static final SwerveUnitConfig lr_Unit_Config = new SwerveUnitConfigBuilder(MOTOR_TYPE.CTRE, 10, MOTOR_TYPE.CTRE, 11, ENCODER_TYPE.CTRE)
                        .setAbsoluteEncoderCANID(18).setCANBUS("canivore").setSlewMotorInverted(false).setLogPIDData(true).build();
        public static final SwerveUnitConfig rf_Unit_Config = new SwerveUnitConfigBuilder(MOTOR_TYPE.CTRE, 14, MOTOR_TYPE.CTRE, 15, ENCODER_TYPE.CTRE)
                        .setAbsoluteEncoderCANID(20).setCANBUS("canivore").setSlewMotorInverted(false).build();
        public static final SwerveUnitConfig rr_Unit_Config = new SwerveUnitConfigBuilder(MOTOR_TYPE.CTRE, 16, MOTOR_TYPE.CTRE, 17, ENCODER_TYPE.CTRE)
                        .setAbsoluteEncoderCANID(21).setCANBUS("canivore").setSlewMotorInverted(false).build();

        /** CAN ID Properties **/
        public static final int GYRO_CAN_ID = 22;

        /** PID Properties **/
        public static final double GYRO_KP = .013, GYRO_KI = .00075, GYRO_KD = .00075, GYRO_MIN = -.5, GYRO_MAX = .5;
        public static final double SLEW_KP = .005, SLEW_KI = .0004, SLEW_KD = -.035, SLEW_KF = 0, SLEW_PID_MIN = -.5, SLEW_PID_MAX = .5;

        /** Auton Mode Constants **/
        public static final String DEFAULT_AUTON = "Disabled";

        /** Auton Modes **/
        public static final String[] AUTON_OPTIONS = { "Test 1", "Test 2", "Test 3" };

}