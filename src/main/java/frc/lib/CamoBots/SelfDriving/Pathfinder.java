package frc.lib.CamoBots.SelfDriving;

import java.io.IOException;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Pathfinder
 * 
 * This subsystem handles self-driving pathfinding and path-following using
 * PathPlannerLib.
 * It uses AutoBuilder to generate commands that drive to a target pose or
 * follow a pre-built path.
 * 
 * All commands are designed to integrate with CommandSwerveDrivetrain and
 * field-centric control.
 * 
 * Usage:
 * - moveToPose(): drive to a Pose2d using pathfinding
 * - moveToPath(): load and follow a PathPlanner-defined path
 * 
 * @author YourName
 * @since 2025 FRC Season
 */
public class Pathfinder extends SubsystemBase {

    // ========== Subsystem Reference ==========


    /**
     * Constructor for Pathfinder
     * 
     * @param m_drive The swerve drive subsystem to use for motion
     */
    public Pathfinder() {}

    // ========== Pathfinding Constraints ==========
    // Max velocity (m/s), acceleration (m/s^2), angular vel/accel (rad/s, rad/s^2)
    public static final  PathConstraints constraints = new PathConstraints(
        3.0, 2.0,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );

    /**
     * Move to a target pose using automatic pathfinding.
     * 
     * @param targetPose  Target field-relative Pose2d (includes rotation)
     * @param endVelocity Target end velocity (m/s) at the final pose
     * @return Command to execute pathfinding to pose
     */
    public static Command moveToPose(Pose2d targetPose, double endVelocity) {
        // Use AutoBuilder to plan a path to the given pose
        return AutoBuilder.pathfindToPose(targetPose, constraints, endVelocity);
    }

    /**
     * Load and follow a pre-planned path by name.
     * 
     * @param PathName Name of the path file created in PathPlanner
     * @return Command to pathfind then follow the loaded path
     * @throws FileVersionException
     * @throws IOException
     * @throws ParseException
     */
    public static Command moveToPath(String PathName)
        throws FileVersionException, IOException, ParseException {

        // Load path from file
        PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);

        // Generate command using AutoBuilder
        return AutoBuilder.pathfindThenFollowPath(path, constraints);
    }

    /**
     * Fallback command if a path could not be recognized.
     * 
     * @param PathName Name of the path that failed to load
     * @return A simple command that prints a warning to console
     */
    public static Command PathNotRecognized(String PathName) {
        return new RunCommand(() ->
            System.out.println("Could not find path: " + PathName)
        );
    }

    // ========== Telemetry Flags ==========

    public static boolean PathEnded = false;

    /**
     * Command to set the PathEnded flag to true.
     * Useful for coordinating multi-step paths or logic resets.
     * 
     * @return Command that flips the PathEnded flag
     */
    public static Command PathEnded() {
        return new RunCommand(() -> PathEnded = true);
    }

    /**
     * Periodic method runs every robot loop.
     * Used here to publish path state to SmartDashboard.
     */
    @Override
    public void periodic() {
       // SmartDashboard.putBoolean("Pathend", PathEnded);
    }
}
