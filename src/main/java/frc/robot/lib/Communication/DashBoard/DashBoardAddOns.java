package frc.robot.lib.Communication.DashBoard;

import java.util.ArrayList;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * ElasticAddOns
 *
 * Utility class to display robot data on SmartDashboard,
 * including color info and swerve drive module states.
 *
 * Features:
 * - Display color as a hex string in NetworkTables.
 * - Display swerve module angles and velocities along with robot pose.
 *
 * Notes:
 * - Assumes valid data arrays passed in for swerve and pose.
 * - Call initializePublisher() before using Color_display.
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class DashBoardAddOns extends SubsystemBase{

    private NetworkTableInstance nt;

    /**
     * Initializes the NetworkTables instance.
     * Must be called before using Color_display.
     */
    public void initializePublisher() {
        nt = NetworkTableInstance.getDefault();
    }

    /**
     * Display a color on the SmartDashboard under a named subtable.
     * Converts Color to hex string representation.
     * 
     * @param identificationName unique name for this color display (e.g. "Robot Status")
     * @param displayColor Color object to display
     */
    public void Color_display(String identificationName, Color displayColor) {
        nt.getTable("Color")
          .getSubTable(identificationName)
          .getEntry(identificationName)
          .setString(displayColor.toHexString());
    }

    /**
     * Display swerve drive data on SmartDashboard.
     * Each module has an angle and velocity pair (total 8 values).
     * Robot angle is displayed as radians (converted from degrees).
     * 
     * @param swerveInput Label to identify this swerve data set on dashboard
     * @param m_moduleArray double array with [angle, velocity] for each of 4 modules (length 8)
     * @param m_poseArray double array containing robot pose info; robot angle at index 2 in degrees
     */
    public static void Swerve_display(String swerveInput, double[] m_moduleArray, double[] m_poseArray) {
        SmartDashboard.putData("Swerve Drive " + swerveInput, new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                // Front Left Module
                builder.addDoubleProperty("Front Left Angle", () -> m_moduleArray[0 * 2], null);
                builder.addDoubleProperty("Front Left Velocity", () -> m_moduleArray[0 * 2 + 1], null);

                // Front Right Module
                builder.addDoubleProperty("Front Right Angle", () -> m_moduleArray[1 * 2], null);
                builder.addDoubleProperty("Front Right Velocity", () -> m_moduleArray[1 * 2 + 1], null);

                // Back Left Module
                builder.addDoubleProperty("Back Left Angle", () -> m_moduleArray[2 * 2], null);
                builder.addDoubleProperty("Back Left Velocity", () -> m_moduleArray[2 * 2 + 1], null);

                // Back Right Module
                builder.addDoubleProperty("Back Right Angle", () -> m_moduleArray[3 * 2], null);
                builder.addDoubleProperty("Back Right Velocity", () -> m_moduleArray[3 * 2 + 1], null);

                // Robot overall angle converted to radians
                builder.addDoubleProperty("Robot Angle", () -> m_poseArray[2], null);
            }
        });
    }

    private final static Field2d field = new Field2d();
    /** Intalizes the SmartDashboard entry Feild and maps PathPlannerLogger to it to auto update pahts */
    public static void FieldDisplay_int() {

        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogActivePathCallback(path -> {
            field.getObject("ActivePath").setPoses(path);
        });

        PathPlannerLogging.setLogTargetPoseCallback(pose -> {
            field.getObject("TargetPose").setPose(pose);
        });

        PathPlannerLogging.setLogCurrentPoseCallback(pose -> {
            field.getObject("PathPlannerPose").setPose(pose);
        });
    }

    /** Update the robot pose on the field display from an array [x, y, angleDegrees] */
    public static void FieldDisplay_UpdateRobotPose(double[] poseArray) {
        if (poseArray.length < 3) {
            System.err.println("Pose array must have at least 3 elements [x, y, angleDegrees]");
            return;
        }
        field.setRobotPose(
            poseArray[0],
            poseArray[1],
            new Rotation2d(poseArray[2])
        );
    }

    /** Clear all tracked objects on the field display */
    public static void FieldDisplay_clear() {
        field.getObject("ActivePath").setPoses(new ArrayList<>());
        field.getObject("TargetPose").setPose(new Pose2d());
        field.getObject("PathPlannerPose").setPose(new Pose2d());
    }

}
