package frc.lib.CamoBots.Logger;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Constants.BuildConstants;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;


public class AdvantageKitLogger {
    
 /**
 * Initializes the AdvantageKit logging system with metadata, data receivers,
 * and command scheduling hooks for logging active commands.
 * <p>
 * This method sets up metadata such as project and build info,
 * configures data receivers differently depending on whether the robot is
 * running in real or simulation mode
 * </p>
 *
 * @param m_robot The robot instance used to determine environment (real vs simulation)
 *                and control timing behavior in simulation.
 * @param REV_PDM_ID ID for the REV power diribution module for logging
 */
    public static void initialize(Robot m_robot, int REV_PDM_ID) {
        DataLogManager.start();

        // Metadata
        Logger.recordMetadata("Code_ProjectName", BuildConstants.PROJECT_NAME);
        Logger.recordMetadata("Code_Version", BuildConstants.VERSION);
        Logger.recordMetadata("Code_BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("Git_Version_Changed", BuildConstants.GIT_REVISION);
        Logger.recordMetadata("Git_Branch", BuildConstants.GIT_BRANCH);

        switch (BuildConstants.DIRTY) {
            case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
            case 1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes");
            default -> Logger.recordMetadata("GitDirty", "Unknown");

        }
                // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                LoggedPowerDistribution.getInstance(REV_PDM_ID, ModuleType.kRev);
            }

            case SIM -> {
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }

            case REPLAY -> {
                // Replaying a log, set up replay source
                m_robot.setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger
                    .setReplaySource(new WPILOGReader(logPath));
                Logger
                    .addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }


        Logger.start();

  }

}
