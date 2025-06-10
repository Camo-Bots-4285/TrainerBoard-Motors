package frc.robot.lib.Communication.NetworkTable_Publishers;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleArrayPublisher;

/**
 * DoubleArrayPublisher
 *
 * Description:
 * This class is used to publish double array data (arrays of floating-point numbers)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send double
 * array information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the double array data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     DoubleArrayPublisher doubleArrayPublisher = new DoubleArrayPublisher("Status", "Drivetrain", "BatteryVoltageArray");
 *   Run in method/command/period:
 *     doubleArrayPublisher.recordDoubleArray(new double[]{12.5, 13.0, 14.5});
 */
public class DoubleArray_Publisher {

    // Declare a DoubleArrayPublisher to handle double array data
    private DoubleArrayPublisher publisher;

    /**
     * Constructor that initializes the DoubleArrayPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Status").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where double array data will be stored (e.g., "BatteryVoltageArray").
     */
    public DoubleArray_Publisher(String categoryName, String identificationName, String entryName) {
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getDoubleArrayTopic(entryName)  // Creates or accesses a double array topic for data.
            .publish();  // Starts publishing the double array data to NetworkTables.
    }

    /**
     * Records a double array value to the specified NetworkTable entry.
     *
     * @param value the double array value to be published.
     */
    public void recordDoubleArray(double[] value) {
        publisher.set(value);  // Publishes the double array value.
    }
}
