package frc.robot.lib.Communication.NetworkTable_Publishers;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

/**
 * StringPublisher
 *
 * Description:
 * This class is used to publish String data (text values)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send string
 * information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the string data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     StringPublisher stringPublisher = new StringPublisher("Status", "Drivetrain", "CurrentState");
 *   Run in method/command/period:
 *     stringPublisher.recordString("Moving");
 */
public class String_Publisher {

    // Declare a StringPublisher to handle String data
    private StringPublisher publisher;

    /**
     * Constructor that initializes the StringPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Status").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where String data will be stored (e.g., "CurrentState").
     */
    public String_Publisher(String categoryName, String identificationName, String entryName) {
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getStringTopic(entryName)  // Creates or accesses a string topic for data.
            .publish();  // Starts publishing the string data to NetworkTables.
    }

    /**
     * Records a String value to the specified NetworkTable entry.
     *
     * @param value the String value to be published.
     */
    public void recordString(String value) {
        publisher.set(value);  // Publishes the string value.
    }
}
