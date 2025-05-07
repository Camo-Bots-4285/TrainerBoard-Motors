package frc.robot.util.Communication.NetworkTable_Publishers;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * BooleanPublisher
 *
 * Description:
 * This class is used to publish boolean data (true/false values)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send boolean
 * information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the boolean data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     BooleanPublisher booleanPublisher = new BooleanPublisher("Status", "Drivetrain", "IsMoving");
 *   Run in method/command/period:
 *     booleanPublisher.recordBoolean(isRobotMoving);
 */
public class Boolean_Publisher {

    // Declare a BooleanPublisher to handle boolean data
    private BooleanPublisher publisher;

    /**
     * Constructor that initializes the BooleanPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Status").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where boolean data will be stored (e.g., "IsMoving").
     */
    public Boolean_Publisher(String categoryName, String identificationName, String entryName) {
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getBooleanTopic(entryName)  // Creates or accesses a boolean topic for data.
            .publish();  // Starts publishing the boolean data to NetworkTables.
    }

    /**
     * Records a boolean value to the specified NetworkTable entry.
     *
     * @param value the boolean value to be published (true or false).
     */
    public void recordBoolean(boolean value) {
        publisher.set(value);  // Publishes the boolean value.
    }
}
