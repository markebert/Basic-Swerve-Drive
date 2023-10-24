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
        public static final SwerveUnitConfig lr_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.CTRE, 11, false,
                        MOTOR_TYPE.CTRE, 10, false,
                        ENCODER_TYPE.CTRE, 18, "canivore");
        public static final SwerveUnitConfig lf_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.CTRE, 13, false,
                        MOTOR_TYPE.CTRE, 12, false,
                        ENCODER_TYPE.CTRE, 19, "canivore");
        public static final SwerveUnitConfig rf_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.CTRE, 15, false,
                        MOTOR_TYPE.CTRE, 14, false,
                        ENCODER_TYPE.CTRE, 20, "canivore");
        public static final SwerveUnitConfig rr_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.CTRE, 17, false,
                        MOTOR_TYPE.CTRE, 16, false,
                        ENCODER_TYPE.CTRE, 21, "canivore");

        /** CAN ID Properties **/
        public static final int GYRO_CAN_ID = 22;

        /** CAN BUS Properties **/
        public static final String GYRO_CAN_BUS = "canivore";

        /** PID Properties **/
        public static final double GYRO_KP = .013, GYRO_KI = .00075, GYRO_KD = .00075, GYRO_MIN = -.5, GYRO_MAX = .5;
        public static final double SLEW_KP = .008, SLEW_KI = .005, SLEW_KD = .005, SLEW_KF = 0, SLEW_PID_MIN = -.5, SLEW_PID_MAX = .5;

        /** Auton Mode Constants **/
        public static final String DEFAULT_AUTON = "Disabled";

        /** Auton Modes **/
        public static final String[] AUTON_OPTIONS = { "Test 1", "Test 2", "Test 3" };

}