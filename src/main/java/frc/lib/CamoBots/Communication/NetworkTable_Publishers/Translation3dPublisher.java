package frc.lib.CamoBots.Communication.NetworkTable_Publishers;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Translation3dPublisher
 *
 * Description:
 * This class is used to publish Translation3d data (robot's position in 3D space)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send Translation3d
 * information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the translation data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     Translation3dPublisher translation3dPublisher = new Translation3dPublisher("Odometry", "Drivetrain", "Translation3dEntry");
 *   Run in method/command/period:
 *     translation3dPublisher.recordTranslation3d(someTranslation3d);
 */
public class Translation3dPublisher {

    // Declare a StructPublisher to handle Translation3d data
    private StructPublisher<Translation3d> publisher;

    /**
     * Constructor that initializes the Translation3dPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Odometry").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where Translation3d data will be stored (e.g., "Translation3dEntry").
     */
    public Translation3dPublisher(String categoryName, String identificationName, String entryName) {
        // Initialize the publisher with the specified NetworkTable category, sub-table, and entry name.
        // The publisher is set up to send Translation3d data to NetworkTables.
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getStructTopic(entryName, Translation3d.struct)  // Creates or accesses a struct topic for Translation3d data.
            .publish();  // Starts publishing the Translation3d data to NetworkTables.
    }

    /**
     * Records a Translation3d value to the specified NetworkTable entry.
     *
     * @param value the Translation3d value to be published (robot's x/y/z position).
     */
    public void recordTranslation3d(Translation3d value) {
        // Sets the value to the publisher, which will send it to the NetworkTable.
        publisher.set(value);  // Publishes the Translation3d value.
    }
}
