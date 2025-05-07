package frc.robot.util.Communication.NetworkTable_Publishers;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanArrayPublisher;

/**
 * BooleanArrayPublisher
 *
 * Description:
 * This class is used to publish boolean array data (arrays of true/false values)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send boolean
 * array information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the boolean array data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     BooleanArrayPublisher booleanArrayPublisher = new BooleanArrayPublisher("Status", "Drivetrain", "IsMovingArray");
 *   Run in method/command/period:
 *     booleanArrayPublisher.recordBooleanArray(new boolean[]{true, false, true});
 */
public class BooleanArray_Publisher {

    // Declare a BooleanArrayPublisher to handle boolean array data
    private BooleanArrayPublisher publisher;

    /**
     * Constructor that initializes the BooleanArrayPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Status").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where boolean array data will be stored (e.g., "IsMovingArray").
     */
    public BooleanArray_Publisher(String categoryName, String identificationName, String entryName) {
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getBooleanArrayTopic(entryName)  // Creates or accesses a boolean array topic for data.
            .publish();  // Starts publishing the boolean array data to NetworkTables.
    }

    /**
     * Records a boolean array value to the specified NetworkTable entry.
     *
     * @param value the boolean array value to be published.
     */
    public void recordBooleanArray(boolean[] value) {
        publisher.set(value);  // Publishes the boolean array value.
    }
}
