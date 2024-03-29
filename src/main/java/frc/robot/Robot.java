// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Java Imports
import java.util.LinkedList;
import java.util.Queue;

// CTRE Imports
import com.ctre.phoenix6.hardware.Pigeon2;

// FRC Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Team 3171 Imports
import frc.team3171.drive.SwerveDrive;
import frc.team3171.protos.AutonData.AutonRecorderData;
import frc.team3171.protos.AutonData.AutonTimestampData;
import frc.team3171.protos.XboxControllerStateOuterClass.XboxControllerState;
import frc.team3171.HelperFunctions;
import frc.team3171.auton.AutonRecorder;
import frc.team3171.controllers.ThreadedPIDController;
import static frc.team3171.HelperFunctions.Generate_Xbox_Controller_State;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot documentation. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot implements RobotProperties {

  // Controllers
  private XboxController driveController;

  // Drive Objects
  private SwerveDrive swerveDrive;
  private Pigeon2 gyro;
  private ThreadedPIDController gyroPIDController;

  // Auton Recorder
  private AutonRecorder autonRecorder;
  private Queue<AutonTimestampData> autonPlaybackQueue;
  private AutonTimestampData playbackData;
  private double autonStartTime;
  private boolean saveNewAuton;

  // Selected Auton String
  private boolean selectedAutonType;
  private String selectedAutonMode;

  // Shuffleboard Choosers
  private SendableChooser<Boolean> autonTypeChooser, fieldOrientationChooser;
  private SendableChooser<String> autonModeChooser;

  // Global Variables
  private volatile boolean fieldOrientationFlipped;

  // Edge Triggers
  private boolean zeroEdgeTrigger;

  @Override
  public void robotInit() {
    // Controllers Init
    driveController = new XboxController(0);

    // Drive Controller Init
    swerveDrive = new SwerveDrive(lr_Unit_Config, lf_Unit_Config, rf_Unit_Config, rr_Unit_Config);

    // Sensors
    gyro = new Pigeon2(GYRO_CAN_ID);
    gyro.reset();

    // PID Controllers
    gyroPIDController = new ThreadedPIDController(() -> Normalize_Gryo_Value(gyro.getAngle() + (fieldOrientationFlipped ? 180 : 0)), GYRO_KP, GYRO_KI, GYRO_KD, GYRO_MIN, GYRO_MAX, false);
    gyroPIDController.setMinSensorValue(-180);
    gyroPIDController.setMaxSensorValue(180);
    gyroPIDController.start();

    // Auton Recorder init
    autonRecorder = new AutonRecorder();
    autonPlaybackQueue = new LinkedList<>();
    playbackData = null;
    saveNewAuton = false;

    // Auton Type init
    selectedAutonType = false;
    autonTypeChooser = new SendableChooser<>();
    autonTypeChooser.setDefaultOption("Playback Auton", false);
    autonTypeChooser.addOption("Record Auton", true);
    SmartDashboard.putData("Auton Type", autonTypeChooser);

    // Field Orientation Chooser
    fieldOrientationChooser = new SendableChooser<>();
    fieldOrientationChooser.setDefaultOption("Pick an option", false);
    fieldOrientationChooser.addOption("0\u00B0", false);
    fieldOrientationChooser.addOption("180\u00B0", true);
    SmartDashboard.putData("Field Orientation Chooser", fieldOrientationChooser);
    SmartDashboard.putBoolean("Flipped", false);

    // Auton Modes init
    selectedAutonMode = DEFAULT_AUTON;
    autonModeChooser = new SendableChooser<>();
    autonModeChooser.setDefaultOption(DEFAULT_AUTON, DEFAULT_AUTON);
    for (final String autonMode : AUTON_OPTIONS) {
      autonModeChooser.addOption(autonMode, autonMode);
    }
    SmartDashboard.putData("Auton Modes", autonModeChooser);

    // Global Variable Init
    fieldOrientationFlipped = false;

    // Edge Trigger Init
    zeroEdgeTrigger = false;

    shuffleboardInit();
  }

  private void shuffleboardInit() {
    ShuffleboardTab periodicTab = Shuffleboard.getTab("Periodic");

    // Auton Selectors
    periodicTab.add("Auton Type", autonTypeChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    periodicTab.add("Auton Modes", autonModeChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    periodicTab.add("Field Orientation Chooser", fieldOrientationChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    periodicTab.addBoolean("Flipped", () -> fieldOrientationFlipped);

    // Put the values on Shuffleboard
    periodicTab.addString("Gyro", () -> String.format("%.2f\u00B0 | %.2f\u00B0", gyroPIDController.getSensorValue(), gyroPIDController.getSensorLockValue()));

    // Controller Values
    swerveDrive.shuffleboardInit("Swerve Debug");
  }

  @Override
  public void robotPeriodic() {
    // Field Orientation Chooser
    fieldOrientationFlipped = fieldOrientationChooser.getSelected().booleanValue();

    // Calibrate Swerve Drive
    final boolean zeroTrigger = driveController.getBackButton() && driveController.getStartButton() && isDisabled();
    if (zeroTrigger && !zeroEdgeTrigger) {
      // Zero the swerve units
      swerveDrive.zero();
      System.out.println("Swerve Drive has been calibrated!");
    }
    zeroEdgeTrigger = zeroTrigger;
  }

  @Override
  public void autonomousInit() {
    // Update Auton Selected Mode and load the auton
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    if (selectedAutonType) {
      playbackData = null;
    } else {
      switch (selectedAutonMode) {
      case DEFAULT_AUTON:
        disabledInit();
        playbackData = null;
        break;
      default:
        final AutonRecorderData autonRecorderData = AutonRecorder.loadFromFile(selectedAutonMode);
        if (autonRecorderData != null) {
          autonPlaybackQueue.clear();
          autonPlaybackQueue.addAll(autonRecorderData.getDataList());
          playbackData = autonPlaybackQueue.poll();
        }
        robotControlsInit();
        break;
      }
    }
    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    switch (selectedAutonMode) {
    case DEFAULT_AUTON:
      disabledPeriodic();
      break;
    default:
      // Plays the recorded auton if theres a valid next step, otherwise disables
      if (playbackData != null) {
        // Get the controller states
        final XboxControllerState driveControllerState = playbackData.getDriverControllerState();
        final XboxControllerState operatorControllerState = playbackData.getOperatorControllerState();

        // Robot drive controls
        robotControlsPeriodic(driveControllerState, operatorControllerState);

        // Checks for new data and when to switch to it
        if ((Timer.getFPGATimestamp() - autonStartTime) >= playbackData.getTimestamp()) {
          playbackData = autonPlaybackQueue.poll();
        }
      } else {
        selectedAutonMode = DEFAULT_AUTON;
        disabledInit();
      }
      break;
    }
  }

  @Override
  public void teleopInit() {
    // Update Auton Selected Mode and reset the data recorder
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    autonRecorder.clear();
    saveNewAuton = selectedAutonType;

    // Reset the robot controls
    robotControlsInit();

    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void teleopPeriodic() {
    // Get the controller states
    final XboxControllerState driveControllerState = Generate_Xbox_Controller_State(driveController);
    final XboxControllerState operatorControllerState = XboxControllerState.getDefaultInstance();

    // Robot drive controls
    robotControlsPeriodic(driveControllerState, operatorControllerState);

    // Auton Recording
    final double autonTimeStamp = Timer.getFPGATimestamp() - autonStartTime;
    if (saveNewAuton && autonTimeStamp <= 15) {
      switch (selectedAutonMode) {
      case DEFAULT_AUTON:
        // Do Nothing
        break;
      default:
        // Adds the recorded data to the auton recorder, but only if the data is new
        var autonTimestampData = AutonTimestampData.newBuilder();
        autonTimestampData.setTimestamp(autonTimeStamp);
        autonTimestampData.setDriverControllerState(driveControllerState);
        autonTimestampData.setOperatorControllerState(operatorControllerState);
        autonRecorder.addNewData(autonTimestampData.build());
        break;
      }
    }
  }

  @Override
  public void disabledInit() {
    // Disable all controllers
    swerveDrive.disable();
    gyroPIDController.disablePID();

    // Once auton recording is done, save the data to a file, if there is any
    if (saveNewAuton) {
      saveNewAuton = false;
      switch (selectedAutonMode) {
      case DEFAULT_AUTON:
        // Do Nothing
        break;
      default:
        autonRecorder.saveToFile(selectedAutonMode);
        break;
      }
    }
  }

  @Override
  public void disabledPeriodic() {
    // Do Nothing
  }

  @Override
  public void testInit() {
    // Do Nothing
  }

  @Override
  public void testPeriodic() {
    // Do Nothing
  }

  private void robotControlsInit() {
    // Reset the drive controller
    swerveDrive.driveInit();
    gyroPIDController.enablePID();
    gyroPIDController.updateSensorLockValue();
  }

  private void robotControlsPeriodic(final XboxControllerState driveControllerState, final XboxControllerState operatorControllerState) {
    // Gyro Value
    final double gyroValue = Normalize_Gryo_Value(gyro.getAngle());

    // Get the needed joystick values after calculating the deadzones
    final double leftStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getLeftX());
    final double leftStickY = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -driveControllerState.getLeftY());
    final double rightStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

    // Calculate the left stick angle and magnitude
    // Calculate the left stick angle and magnitude
    final double leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
    double leftStickMagnitude;
    leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
    if (leftStickMagnitude > 1.0) {
      leftStickMagnitude = 1;
    }

    // Calculate the field corrected drive angle
    final double fieldCorrectedAngle = FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(leftStickAngle - gyroValue) : leftStickAngle;

    // Drive Controls
    final boolean boostMode = driveControllerState.getLeftBumper() || driveControllerState.getRightBumper();
    if (rightStickX != 0) {
      // Manual turning
      gyroPIDController.disablePID();
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, rightStickX, boostMode);
    } else {
      // Normal gyro locking
      gyroPIDController.enablePID();

      // Quick Turning
      if (driveControllerState.hasPov() && driveControllerState.getPov() != -1) {
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(driveControllerState.getPov()));
      } else if (driveControllerState.getYButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(0);
      } else if (driveControllerState.getBButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(90);
      } else if (driveControllerState.getAButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(180);
      } else if (driveControllerState.getXButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(-90);
      }
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? gyroPIDController.getPIDValue() : 0, boostMode);
    }

  }

}
