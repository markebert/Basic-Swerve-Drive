package frc.team3171.auton;

// Java Imports
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;

// Google Imports
import com.google.protobuf.CodedInputStream;

import frc.team3171.protos.AutonData.AutonRecorderData;
import frc.team3171.protos.AutonData.AutonRecorderData.Builder;
import frc.team3171.protos.AutonData.AutonTimestampData;

// Team 3171 Imports
/**
 * @author Mark Ebert
 * @author Charles Fee
 * @author Elijah Hoda
 */
public class AutonRecorder {

    private final Builder autonRecorderData;

    /**
     * Constructor
     */
    public AutonRecorder() {
        autonRecorderData = AutonRecorderData.newBuilder();
    }

    /**
     * 
     * @param newData
     */
    public void addNewData(final AutonTimestampData newData) {
        final int autonRecorderDataCount = autonRecorderData.getDataCount();
        if (newData != null && autonRecorderDataCount > 1) {
            if (isChanged(autonRecorderData.getData(autonRecorderDataCount - 1), newData)) {
                autonRecorderData.addData(newData);
            }
        } else {
            autonRecorderData.addData(newData);
        }
    }

    /**
     * Clears whatever the AutonRecorder currently has saved.
     */
    public void clear() {
        autonRecorderData.clearData();
    }

    /**
     * Saves the current data collected by the auton recorder to the specified file path and clears the AutonRecorder.
     * 
     * @param autonFileName The path to the file to save the auton data to.
     */
    public void saveToFile(final String autonFileName) {
        try {
            // Check if the auton file exists, if it does delete then file and recreate it
            File autonFile = new File(String.format("/home/lvuser/%s.txt", autonFileName));
            if (autonFile.exists()) {
                autonFile.delete();
            }
            autonFile.createNewFile();

            // Create a buffered writer and write the contents of the auton recorder
            try (BufferedWriter writer = new BufferedWriter(new FileWriter(autonFile))) {
                AutonRecorderData recordedData = autonRecorderData.build();
                writer.write(recordedData.toString());
                writer.flush();
            }
            System.out.println("Auton Successfully Saved!");
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            autonRecorderData.clearData();
        }
    }

    /**
     * 
     * @param autonFileName
     * @return
     */
    public static AutonRecorderData loadFromFile(final String autonFileName) {
        AutonRecorderData autonRecordedData;
        try (FileInputStream autonFile = new FileInputStream(String.format("/home/lvuser/%s.txt", autonFileName))) {
            // Gets the saved auton file as a coded input stream
            CodedInputStream inputStream = CodedInputStream.newInstance(autonFile);
            // Parses the data from the stream into the respecitve protobuf object
            autonRecordedData = AutonRecorderData.parseFrom(inputStream);
            System.out.println("Auton Successfully Loaded!");
        } catch (Exception e) {
            autonRecordedData = AutonRecorderData.newBuilder().build();
            System.err.printf("The Auton File %s.txt could not be found!\n", autonFileName);
        }
        return autonRecordedData;
    }

    /**
     * Checks if the provided new data is different from the current data, as long as the new data's timestamp is later then the current timestamp, if so returns true, otherwise false.
     * 
     * @param newData The new set of data containing the new values to compare to the current values.
     * @return Returns true if the new data is different, otherwise false.
     */
    public boolean isChanged(final AutonTimestampData oldData, final AutonTimestampData newData) {
        if (newData.getTimestamp() >= oldData.getTimestamp()) {
            return !newData.equals(oldData) || !newData.getDriverControllerState().equals(oldData.getDriverControllerState())
                    || !newData.getOperatorControllerState().equals(oldData.getOperatorControllerState());
        }
        return false;
    }

}
