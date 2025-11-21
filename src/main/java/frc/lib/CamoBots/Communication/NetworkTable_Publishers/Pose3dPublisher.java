package frc.lib.CamoBots.Communication.NetworkTable_Publishers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Pose3dPublisher
 *
 * Description:
 * This class is used to publish Pose3d data (robot's position and orientation in 3D space)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send Pose3d
 * information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the pose data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     Pose3dPublisher pose3dPublisher = new Pose3dPublisher("Odometry", "Drivetrain", "Pose3dEntry");
 *   Run in method/command/period:
 *     pose3dPublisher.recordPose3d(somePose3d);
 */
public class Pose3dPublisher {

    // Declare a StructPublisher to handle Pose3d data
    private StructPublisher<Pose3d> publisher;

    /**
     * Constructor that initializes the Pose3dPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Odometry").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where Pose3d data will be stored (e.g., "Pose3dEntry").
     */
    public Pose3dPublisher(String categoryName, String identificationName, String entryName) {
        // Initialize the publisher with the specified NetworkTable category, sub-table, and entry name.
        // The publisher is set up to send Pose3d data to NetworkTables.
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getStructTopic(entryName, Pose3d.struct)  // Creates or accesses a struct topic for Pose3d data.
            .publish();  // Starts publishing the Pose3d data to NetworkTables.
    }

    /**
     * Records a Pose3d value to the specified NetworkTable entry.
     *
     * @param value the Pose3d value to be published (robot's x/y/z position and orientation).
     */
    public void recordPose3d(Pose3d value) {
        // Sets the value to the publisher, which will send it to the NetworkTable.
        publisher.set(value);  // Publishes the Pose3d value.
    }
}
