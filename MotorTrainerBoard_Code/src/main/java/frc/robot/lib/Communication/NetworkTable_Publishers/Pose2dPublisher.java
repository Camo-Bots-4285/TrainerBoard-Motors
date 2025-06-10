package frc.robot.lib.Communication.NetworkTable_Publishers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Pose2dPublisher
 *
 * Description:
 * This class is used to publish Pose2d data (robot's position and orientation in 2D space)
 * to a specified NetworkTable entry. It uses WPILib's NetworkTables to send Pose2d
 * information to other components, such as the driver station or other subsystems.
 *
 * Notes:
 * - This class should be initialized with the appropriate category, sub-table, and entry
 *   name for the pose data. The publisher is created once and reused for the lifetime of the object.
 *
 * Example Usage:
 *   Place in class outside methods:
 *     Pose2dPublisher pose2dPublisher = new Pose2dPublisher("Odometry", "Drivetrain", "Pose2dEntry");
 *   Run in method/command/period:
 *     pose2dPublisher.recordPose2d(somePose2d);
 */
public class Pose2dPublisher {

    // Declare a StructPublisher to handle Pose2d data
    private StructPublisher<Pose2d> publisher;

    /**
     * Constructor that initializes the Pose2dPublisher with a specific category, sub-table, and entry name.
     * 
     * @param categoryName       the name of the top-level table (e.g., "Odometry").
     * @param identificationName the name of the sub-table (e.g., "Drivetrain").
     * @param entryName          the name of the entry where Pose2d data will be stored (e.g., "Pose2dEntry").
     */
    public Pose2dPublisher(String categoryName, String identificationName, String entryName) {
        // Initialize the publisher with the specified NetworkTable category, sub-table, and entry name.
        // The publisher is set up to send Pose2d data to NetworkTables.
        this.publisher = NetworkTableInstance.getDefault()  // Accesses the default instance of NetworkTables.
            .getTable(categoryName)  // Retrieves the top-level table (category).
            .getSubTable(identificationName)  // Retrieves the sub-table (e.g., "Drivetrain").
            .getStructTopic(entryName, Pose2d.struct)  // Creates or accesses a struct topic for Pose2d data.
            .publish();  // Starts publishing the Pose2d data to NetworkTables.
    }

    /**
     * Records a Pose2d value to the specified NetworkTable entry.
     *
     * @param value the Pose2d value to be published (robot's x/y position and heading).
     */
    public void recordPose2d(Pose2d value) {
        // Sets the value to the publisher, which will send it to the NetworkTable.
        publisher.set(value);  // Publishes the Pose2d value.
    }
}
