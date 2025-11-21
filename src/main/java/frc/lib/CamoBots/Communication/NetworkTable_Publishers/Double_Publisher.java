package frc.lib.CamoBots.Communication.NetworkTable_Publishers;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;

/**
 * DoublePublisher
 *
 * Description:
 * This class is used to publish double data (floating-point numbers)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send double
 * information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the double data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     DoublePublisher doublePublisher = new DoublePublisher("Status", "Drivetrain", "BatteryVoltage");
 *   Run in method/command/period:
 *     doublePublisher.recordDouble(12.5);
 */
public class Double_Publisher {

    // Declare a DoublePublisher to handle double data
    private DoublePublisher publisher;

    /**
     * Constructor that initializes the DoublePublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Status").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where double data will be stored (e.g., "BatteryVoltage").
     */
    public Double_Publisher(String categoryName, String identificationName, String entryName) {
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getDoubleTopic(entryName)  // Creates or accesses a double topic for data.
            .publish();  // Starts publishing the double data to NetworkTables.
    }

    /**
     * Records a double value to the specified NetworkTable entry.
     *
     * @param value the double value to be published.
     */
    public void recordDouble(double value) {
        publisher.set(value);  // Publishes the double value.
    }
}
