package frc.robot.lib.Communication.NetworkTable_Publishers;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.IntegerArrayPublisher;

/**
 * IntArrayPublisher
 *
 * Description:
 * This class is used to publish integer array data (arrays of integer values)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send integer
 * array information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the integer array data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     IntArrayPublisher intArrayPublisher = new IntArrayPublisher("Status", "Drivetrain", "SpeedArray");
 *   Run in method/command/period:
 *     intArrayPublisher.recordIntArray(new int[]{10, 15, 20});
 */
public class IntegerArray_Publisher {

    // Declare an IntegerArrayPublisher to handle integer array data
    private IntegerArrayPublisher publisher;

    /**
     * Constructor that initializes the IntArrayPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Status").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where integer array data will be stored (e.g., "SpeedArray").
     */
    public IntegerArray_Publisher(String categoryName, String identificationName, String entryName) {
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getIntegerArrayTopic(entryName)  // Creates or accesses an integer array topic for data.
            .publish();  // Starts publishing the integer array data to NetworkTables.
    }

    /**
     * Records an integer array value to the specified NetworkTable entry.
     *
     * @param value the integer array value to be published.
     */
    public void recordIntArray(long[] value) {
        publisher.set(value);  // Publishes the integer array value.
    }
}
