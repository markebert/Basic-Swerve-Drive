package frc.robot;

// Team 3171 Imports
import frc.team3171.drive.SwerveUnitConfig;
import frc.team3171.drive.SwerveUnitConfig.ENCODER_TYPE;
import frc.team3171.drive.SwerveUnitConfig.MOTOR_TYPE;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

        /** Debug Info **/
        public static final boolean DEBUG = true;

        /** Drive Variables **/
        public static final boolean FIELD_ORIENTED_SWERVE = true;
        public static final double JOYSTICK_DEADZONE = .08;
        public static final double SLOW_DRIVE_SPEED = .6, MAX_ROTATION_SPEED = .6;

        /** Swerve Unit Configuration **/
        public static final SwerveUnitConfig lr_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.REV, 5, false,
                        MOTOR_TYPE.CTRE, 4, false, ENCODER_TYPE.REV, 18);
        public static final SwerveUnitConfig lf_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.REV, 3, false,
                        MOTOR_TYPE.CTRE, 2, false, ENCODER_TYPE.REV, 19);
        public static final SwerveUnitConfig rf_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.REV, 7, false,
                        MOTOR_TYPE.CTRE, 6, false, ENCODER_TYPE.REV, 20);
        public static final SwerveUnitConfig rr_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.REV, 9, false,
                        MOTOR_TYPE.CTRE, 8, false, ENCODER_TYPE.REV, 21);

        /** CAN ID Properties **/
        public static final int GYRO_CAN_ID = 10;

        /** PID Properties **/
        public static final double GYRO_KP = .013, GYRO_KI = .00075, GYRO_KD = .00075, GYRO_MIN = -.5, GYRO_MAX = .5;
        public static final double SLEW_KP = 0, SLEW_KI = 0, SLEW_KD = 0, SLEW_KF = 0, SLEW_PID_MIN = -.5, SLEW_PID_MAX = .5;

        /** Auton Mode Constants **/
        public static final String DEFAULT_AUTON = "Disabled";

        /** Auton Modes **/
        public static final String[] AUTON_OPTIONS = { "Test 1", "Test 2", "Test 3" };

}