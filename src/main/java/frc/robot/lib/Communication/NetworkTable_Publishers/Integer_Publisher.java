package frc.robot.lib.Communication.NetworkTable_Publishers;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.IntegerPublisher;

/**
 * IntPublisher
 *
 * Description:
 * This class is used to publish integer data (whole numbers)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send integer
 * information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the integer data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     IntPublisher intPublisher = new IntPublisher("Status", "Drivetrain", "Speed");
 *   Run in method/command/period:
 *     intPublisher.recordInt(10);
 */
public class Integer_Publisher {

    // Declare an IntegerPublisher to handle integer data
    private IntegerPublisher publisher;

    /**
     * Constructor that initializes the IntPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Status").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where integer data will be stored (e.g., "Speed").
     */
    public Integer_Publisher(String categoryName, String identificationName, String entryName) {
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getIntegerTopic(entryName)  // Creates or accesses an integer topic for data.
            .publish();  // Starts publishing the integer data to NetworkTables.
    }

    /**
     * Records an integer value to the specified NetworkTable entry.
     *
     * @param value the integer value to be published.
     */
    public void recordInt(int value) {
        publisher.set(value);  // Publishes the integer value.
    }
}
