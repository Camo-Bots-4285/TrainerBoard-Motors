package frc.robot.util.Communication.NetworkTable_Publishers;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;

/**
 * StringArrayPublisher
 *
 * Description:
 * This class is used to publish string array data (arrays of text values)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send string
 * array information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the string array data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     StringArrayPublisher stringArrayPublisher = new StringArrayPublisher("Status", "Drivetrain", "StateArray");
 *   Run in method/command/period:
 *     stringArrayPublisher.recordStringArray(new String[]{"Moving", "Idle", "Charging"});
 */
public class StringArray_Publisher {

    // Declare a StringArrayPublisher to handle string array data
    private StringArrayPublisher publisher;

    /**
     * Constructor that initializes the StringArrayPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Status").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where string array data will be stored (e.g., "StateArray").
     */
    public StringArray_Publisher(String categoryName, String identificationName, String entryName) {
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getStringArrayTopic(entryName)  // Creates or accesses a string array topic for data.
            .publish();  // Starts publishing the string array data to NetworkTables.
    }

    /**
     * Records a string array value to the specified NetworkTable entry.
     *
     * @param value the string array value to be published.
     */
    public void recordStringArray(String[] value) {
        publisher.set(value);  // Publishes the string array value.
    }
}
