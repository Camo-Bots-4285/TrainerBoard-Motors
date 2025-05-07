package frc.robot.util.Communication.NetworkTable_Publishers;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Rotation3dPublisher
 *
 * Description:
 * This class is used to publish Rotation3d data (robot's rotation in 3D space)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send Rotation3d
 * information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the rotation data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     Rotation3dPublisher rotation3dPublisher = new Rotation3dPublisher("Odometry", "Drivetrain", "Rotation3dEntry");
 *   Run in method/command/period:
 *     rotation3dPublisher.recordRotation3d(someRotation3d);
 */
public class Rotation3dPublisher {

    // Declare a StructPublisher to handle Rotation3d data
    private StructPublisher<Rotation3d> publisher;

    /**
     * Constructor that initializes the Rotation3dPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Odometry").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where Rotation3d data will be stored (e.g., "Rotation3dEntry").
     */
    public Rotation3dPublisher(String categoryName, String identificationName, String entryName) {
        // Initialize the publisher with the specified NetworkTable category, sub-table, and entry name.
        // The publisher is set up to send Rotation3d data to NetworkTables.
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getStructTopic(entryName, Rotation3d.struct)  // Creates or accesses a struct topic for Rotation3d data.
            .publish();  // Starts publishing the Rotation3d data to NetworkTables.
    }

    /**
     * Records a Rotation3d value to the specified NetworkTable entry.
     *
     * @param value the Rotation3d value to be published (robot's rotation in 3D space).
     */
    public void recordRotation3d(Rotation3d value) {
        // Sets the value to the publisher, which will send it to the NetworkTable.
        publisher.set(value);  // Publishes the Rotation3d value.
    }
}
