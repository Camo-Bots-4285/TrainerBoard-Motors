package frc.lib.CamoBots.Communication.NetworkTable_Publishers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Translation2dPublisher
 *
 * Description:
 * This class is used to publish Translation2d data (robot's position in 2D space)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send Translation2d
 * information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the translation data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     Translation2dPublisher translation2dPublisher = new Translation2dPublisher("Odometry", "Drivetrain", "Translation2dEntry");
 *   Run in method/command/period:
 *     translation2dPublisher.recordTranslation2d(someTranslation2d);
 */
public class Translation2dPublisher {

    // Declare a StructPublisher to handle Translation2d data
    private StructPublisher<Translation2d> publisher;

    /**
     * Constructor that initializes the Translation2dPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Odometry").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where Translation2d data will be stored (e.g., "Translation2dEntry").
     */
    public Translation2dPublisher(String categoryName, String identificationName, String entryName) {
        // Initialize the publisher with the specified NetworkTable category, sub-table, and entry name.
        // The publisher is set up to send Translation2d data to NetworkTables.
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getStructTopic(entryName, Translation2d.struct)  // Creates or accesses a struct topic for Translation2d data.
            .publish();  // Starts publishing the Translation2d data to NetworkTables.
    }

    /**
     * Records a Translation2d value to the specified NetworkTable entry.
     *
     * @param value the Translation2d value to be published (robot's x/y position).
     */
    public void recordTranslation2d(Translation2d value) {
        // Sets the value to the publisher, which will send it to the NetworkTable.
        publisher.set(value);  // Publishes the Translation2d value.
    }
}
