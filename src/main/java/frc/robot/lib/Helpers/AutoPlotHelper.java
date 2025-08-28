package frc.robot.lib.Helpers;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.Communication.DigitalButtons.CustomAutoBuilder;

public class AutoPlotHelper {

    // Field2d instance used for plotting (pre-match preview)
    private static final Field2d field = new Field2d();

    /** Initialize: Put Field2d on SmartDashboard */
    public static void init() {
        SmartDashboard.putData("Auto Preview Field", field);
    }

    /**
     * Plot either a full PathPlanner auto or a custom sequence from button selections.
     * @param selectedAutoName The auto name from chooser (e.g. "3 Piece" or "Custom")
     * @param customButtons ButtonSequence (used only if autoName == "Custom")
     */
    public static void plotAutoOrCustom(String selectedAutoName, CustomAutoBuilder customButtons) {
        if (!"Custom".equalsIgnoreCase(selectedAutoName)) {
            plotAuto(selectedAutoName);
        } else {
            plotCustomAuto(customButtons);
        }
    }

    /** Plot a full auto using all paths inside the auto file */
    public static void plotAuto(String autoName) {
        try {
            List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
            List<Pose2d> poses = new ArrayList<>();

            for (PathPlannerPath path : paths) {
                poses.addAll(path.getPathPoses()); // Reuse internal method
            }

            field.getObject("PlottedAuto").setPoses(poses);
        } catch (Exception e) {
            System.err.println("Failed to load or parse auto: " + autoName);
            e.printStackTrace();
        }
    }

    /** Get poses from a single PathPlanner path by name */
    private static List<Pose2d> getPathPoses(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return path.getPathPoses();
        } catch (Exception e) {
            System.err.println("Failed to load path: " + pathName);
            return new ArrayList<>();
        }
    }

    /** Plot a single path by name */
    public static void plotPath(String pathName) {
        List<Pose2d> poses = getPathPoses(pathName);
        field.getObject("PlottedPath").setPoses(poses);
    }

    /** Plot a custom auto sequence from ButtonSequence */
    private static void plotCustomAuto(CustomAutoBuilder customButtons) {
        List<String> pathNames = CustomAutoBuilder.buildPathSequence(customButtons.getSelectedButtonNames());
        List<Pose2d> allPoses = new ArrayList<>();

        for (String pathName : pathNames) {
            allPoses.addAll(getPathPoses(pathName));
        }

        field.getObject("PlottedAuto").setPoses(allPoses);
    }

    /** Clear all plotted poses from field */
    public static void clear() {
        field.getObject("PlottedAuto").setPoses(new ArrayList<>());
        field.getObject("PlottedPath").setPoses(new ArrayList<>());
    }

    /** Update robot pose on the field */
    public static void updateRobotPose(double[] poseArray) {
        if (poseArray.length >= 3) {
            field.setRobotPose(
                poseArray[0],
                poseArray[1],
                Rotation2d.fromDegrees(poseArray[2])
            );
        }
    }

    /** Accessor for Field2d if needed externally */
    public static Field2d getField() {
        return field;
    }
}
