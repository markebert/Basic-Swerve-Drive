package frc.robot;

// Team 3171 Imports
import frc.team3171.protos.SlewDrive.Encoder_Type;
import frc.team3171.protos.SlewDrive.Motor_Type;
import frc.team3171.protos.SlewDrive.SlewUnitConfiguration;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

        /** Debug Options **/
        public static final boolean DEBUG = true;
        public static final boolean SWERVE_DIRECTION_DEBUG = false;
        public static final String PID_LOG_ADDRESS = "10.31.71.201";

        /** Drive Variables **/
        public static final boolean FIELD_ORIENTED_SWERVE = true;
        public static final double JOYSTICK_DEADZONE = .08;
        public static final double MAX_DRIVE_SPEED = .5, MAX_ROTATION_SPEED = .6;
        public static final boolean PINWHEEL_ZERO_ORIENTATION = true;
        public static final boolean SWERVE_UNIT_ORIENTATION_OPTIMIZATION = true;

        /** Swerve Unit Configuration **/
        public static final SlewUnitConfiguration lf_Unit_Config = SlewUnitConfiguration.newBuilder().setDriveMotorType(Motor_Type.CTRE_FALCON).setDriveMotorID(2)
                        .setSlewMotorType(Motor_Type.CTRE_FALCON).setSlewMotorID(3).setAbsoluteEncoderType(Encoder_Type.CTRE_CANCODER).setAbsoluteEncoderID(10).build();
        public static final SlewUnitConfiguration lr_Unit_Config = SlewUnitConfiguration.newBuilder().setDriveMotorType(Motor_Type.CTRE_FALCON).setDriveMotorID(4)
                        .setSlewMotorType(Motor_Type.CTRE_FALCON).setSlewMotorID(5).setAbsoluteEncoderType(Encoder_Type.CTRE_CANCODER).setAbsoluteEncoderID(11).build();
        public static final SlewUnitConfiguration rf_Unit_Config = SlewUnitConfiguration.newBuilder().setDriveMotorType(Motor_Type.CTRE_FALCON).setDriveMotorID(6)
                        .setSlewMotorType(Motor_Type.CTRE_FALCON).setSlewMotorID(7).setAbsoluteEncoderType(Encoder_Type.CTRE_CANCODER).setAbsoluteEncoderID(12).build();
        public static final SlewUnitConfiguration rr_Unit_Config = SlewUnitConfiguration.newBuilder().setDriveMotorType(Motor_Type.CTRE_FALCON).setDriveMotorID(8)
                        .setSlewMotorType(Motor_Type.CTRE_FALCON).setSlewMotorID(9).setAbsoluteEncoderType(Encoder_Type.CTRE_CANCODER).setAbsoluteEncoderID(13).build();

        /** CAN ID Properties **/
        public static final int GYRO_CAN_ID = 10;

        /** PID Properties **/
        public static final double GYRO_KP = .013, GYRO_KI = .00075, GYRO_KD = .00075, GYRO_MIN = -.5, GYRO_MAX = .5;
        public static final double SLEW_KP = -.005, SLEW_KI = -.0004, SLEW_KD = .035, SLEW_KF = 0, SLEW_PID_MIN = -1, SLEW_PID_MAX = 1;

        /** Auton Mode Constants **/
        public static final String DEFAULT_AUTON = "Disabled";

        /** Auton Modes **/
        public static final String[] AUTON_OPTIONS = { "Test 1", "Test 2", "Test 3" };

}