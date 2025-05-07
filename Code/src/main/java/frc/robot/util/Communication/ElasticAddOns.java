package frc.robot.util.Communication;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * ElasticAddOns
 *
 * Description:
 * This utility class provides methods for displaying information on the SmartDashboard,
 * including robot color information and swerve drive data. It allows easy integration of 
 * color display and robot state visualization for debugging and monitoring.
 *
 * Features:
 * - Display robot color as a hexadecimal value on the SmartDashboard.
 * - Display real-time swerve drive data, including angles and velocities of all swerve modules.
 * - Display robot's overall pose information (angle) for visualization.
 *
 * Notes:
 * - This class interacts with SmartDashboard and assumes the usage of the `Sendable` interface for custom data visualization.
 * - Ensure that the `m_moduleArray` and `m_poseArray` are populated with valid data before calling the display methods.
 */
public class ElasticAddOns {

        // Declare the NetworkTableInstance (but don't initialize here)
        private NetworkTableInstance nt;

        /**
         * Initializes the NetworkTables instance.
         * This will set up the NetworkTableInstance and ensure it's ready for use.
         */
        public void initializePublisher() {
            // Initialize the NetworkTable instance
            this.nt = NetworkTableInstance.getDefault();
        }

    /**
     * Displays a color on the SmartDashboard with a specific identification name.
     * The color is converted to its hexadecimal representation for easier readability.
     * This is useful for showing robot status or LED color information on the dashboard.
     * 
     * @param identificationName the unique name to identify the color (e.g., "Robot Status")
     * @param displayColor the `Color` object to be displayed on the SmartDashboard
     */
    public void Color_display(String identificationName, Color displayColor){
        // Convert the Color object to a hex string and put it on the SmartDashboard under the "Color" key.
        nt.getTable("Color").getSubTable(identificationName)
        .getEntry(identificationName).setString( displayColor.toHexString());
        
    }

    /**
     * Displays swerve drive module data (angle and velocity) along with the robot's overall angle 
     * on the SmartDashboard. This method creates a custom `Sendable` object to send the module's 
     * information to the dashboard for real-time monitoring.
     * 
     * The method assumes the following structure for the input arrays:
     * - m_moduleArray contains angle and velocity pairs for each of the four swerve modules (angle, velocity).
     * - m_poseArray contains the robot's pose information, with the robot's angle typically at index 2.
     *
     * @param SwerveInput a unique string identifier used to label the swerve data on the dashboard (e.g., "Drive", "Test")
     * @param m_moduleArray an array containing the angle and velocity for each swerve module (length should be 8, 2 elements per module)
     * @param m_poseArray an array containing the robot's overall pose, with the robot's angle at index 2
     */
    public static void Swerve_display(String SwerveInput, double[] m_moduleArray, double[] m_poseArray) {
        // Create a custom Sendable object to send the swerve drive data to SmartDashboard.
        SmartDashboard.putData("Swerve Drive " + SwerveInput, new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                // Set the type of the Sendable object for SmartDashboard to interpret it as a "SwerveDrive" type.
                builder.setSmartDashboardType("SwerveDrive");
    
                // Add properties for the front-left swerve module's angle and velocity.
                builder.addDoubleProperty("Front Left Angle", () -> m_moduleArray[0 * 2 + 0], null);  // Angle for front-left module
                builder.addDoubleProperty("Front Left Velocity", () -> m_moduleArray[0 * 2 + 1], null);  // Velocity for front-left module
    
                // Add properties for the front-right swerve module's angle and velocity.
                builder.addDoubleProperty("Front Right Angle", () -> m_moduleArray[1 * 2 + 0], null);  // Angle for front-right module
                builder.addDoubleProperty("Front Right Velocity", () -> m_moduleArray[1 * 2 + 1], null);  // Velocity for front-right module
    
                // Add properties for the back-left swerve module's angle and velocity.
                builder.addDoubleProperty("Back Left Angle", () -> m_moduleArray[2 * 2 + 0], null);  // Angle for back-left module
                builder.addDoubleProperty("Back Left Velocity", () -> m_moduleArray[2 * 2 + 1], null);  // Velocity for back-left module
    
                // Add properties for the back-right swerve module's angle and velocity.
                builder.addDoubleProperty("Back Right Angle", () -> m_moduleArray[3 * 2 + 0], null);  // Angle for back-right module
                builder.addDoubleProperty("Back Right Velocity", () -> m_moduleArray[3 * 2 + 1], null);  // Velocity for back-right module
    
                // Add the robot's overall angle to the SmartDashboard.
                builder.addDoubleProperty("Robot Angle", () -> m_poseArray[2], null);  // Robot's angle (pose information)
            }
        });
    }
}
