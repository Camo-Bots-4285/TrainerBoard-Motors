package frc.lib.CamoBots.Communication.NetworkTable_Publishers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Rotation2dPublisher
 *
 * Description:
 * This class is used to publish Rotation2d data (robot's rotation in 2D space)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send Rotation2d
 * information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the rotation data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     Rotation2dPublisher rotation2dPublisher = new Rotation2dPublisher("Odometry", "Drivetrain", "Rotation2dEntry");
 *   Run in method/command/period:
 *     rotation2dPublisher.recordRotation2d(someRotation2d);
 */
public class Rotation2dPublisher {

    // Declare a StructPublisher to handle Rotation2d data
    private StructPublisher<Rotation2d> publisher;

    /**
     * Constructor that initializes the Rotation2dPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Odometry").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where Rotation2d data will be stored (e.g., "Rotation2dEntry").
     */
    public Rotation2dPublisher(String categoryName, String identificationName, String entryName) {
        // Initialize the publisher with the specified NetworkTable category, sub-table, and entry name.
        // The publisher is set up to send Rotation2d data to NetworkTables.
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getStructTopic(entryName, Rotation2d.struct)  // Creates or accesses a struct topic for Rotation2d data.
            .publish();  // Starts publishing the Rotation2d data to NetworkTables.
    }

    /**
     * Records a Rotation2d value to the specified NetworkTable entry.
     *
     * @param value the Rotation2d value to be published (robot's rotation).
     */
    public void recordRotation2d(Rotation2d value) {
        // Sets the value to the publisher, which will send it to the NetworkTable.
        publisher.set(value);  // Publishes the Rotation2d value.
    }
}
