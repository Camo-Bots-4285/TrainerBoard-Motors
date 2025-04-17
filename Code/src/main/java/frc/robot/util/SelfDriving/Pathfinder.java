package frc.robot.util.SelfDriving;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Pathfinder extends SubsystemBase{

        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));
    
    public Pathfinder(){}

    public Command moveToPose(Pose2d targetPose, double endVelocity){
    // Since we are using a holonomic drivetrain, the rotation component of this pose
    // represents the goal holonomic rotation

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        endVelocity
        );

        return pathfindingCommand;
    }

   public Command moveToPath(String PathName) throws FileVersionException, IOException, ParseException{

   // Load the path we want to pathfind to and follow
    PathPlannerPath path = PathPlannerPath.fromPathFile("PathName");

   // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints);

    return pathfindingCommand;

    
    }

    
}
