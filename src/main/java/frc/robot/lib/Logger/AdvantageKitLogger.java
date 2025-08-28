package frc.robot.lib.Logger;

import java.util.HashSet;
import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.Constants.BuildConstants;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class AdvantageKitLogger {
    private final static Set<Command> activeCommands = new HashSet<>();
    
 /**
 * Initializes the AdvantageKit logging system with metadata, data receivers,
 * and command scheduling hooks for logging active commands.
 * <p>
 * This method sets up metadata such as project and build info,
 * configures data receivers differently depending on whether the robot is
 * running in real or simulation mode, and registers listeners with the
 * CommandScheduler to track active commands for logging purposes.
 * </p>
 *
 * @param m_robot The robot instance used to determine environment (real vs simulation)
 *                and control timing behavior in simulation.
 */
    public static void initialize(Robot m_robot) {
        DataLogManager.start();

        // Metadata
        Logger.recordMetadata("Code_ProjectName", BuildConstants.PROJECT_NAME);
        Logger.recordMetadata("Code_Version", BuildConstants.VERSION);
        Logger.recordMetadata("Code_BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("Git_Version_Changed", BuildConstants.GIT_REVISION);
        Logger.recordMetadata("Git_Branch", BuildConstants.GIT_BRANCH);

        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to USB stick
            Logger.addDataReceiver(new NT4Publisher()); // Publish to NetworkTables
            LoggedPowerDistribution.getInstance(50, ModuleType.kRev); // Example CAN ID 50
        } else {
            // m_robot.setUseTiming(false); // Run as fast as possible

            // String logPath = LogFileUtil.findReplayLog(); // Replay log path
            // Logger.setReplaySource(new WPILOGReader(logPath));
            // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));

            Logger.addDataReceiver(new WPILOGWriter()); // Log to USB stick
            Logger.addDataReceiver(new NT4Publisher()); // Publish to NetworkTables
        }

        Logger.start();
    
        CommandScheduler scheduler = CommandScheduler.getInstance();
    
        scheduler.onCommandInitialize(command -> {
            activeCommands.add(command);
            Logger.recordOutput("6-Commands/" + command.getName(), true);
        });
        scheduler.onCommandFinish(command -> {
            activeCommands.remove(command);
            Logger.recordOutput("6-Commands/" + command.getName(), false);
        });
        scheduler.onCommandInterrupt(command -> {
            activeCommands.remove(command);
        Logger.recordOutput("6-Commands/" + command.getName(), false);
    });
  }

    /**
     * Periodically updates the AdvantageKit logger with the status of currently
     * active commands.
     * <p>
     * This method should be called regularly (e.g., in robotPeriodic) to
     * continually record which commands are currently active, enabling
     * more detailed command activity logging for debugging and replay.
     * </p>
     */
    public static void periodicUpdate() {
        for (Command command : activeCommands) {
            Logger.recordOutput("6-Commands/" + command.getName(), true);
        }
    }

}
