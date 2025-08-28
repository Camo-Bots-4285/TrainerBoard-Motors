package frc.robot.lib.Communication.DashBoard;

import frc.robot.lib.Communication.DashBoard.Elastic.Notification.NotificationLevel;
import frc.robot.lib.Communication.DashBoard.QFRCLib.ErrorLevel;


/**
 * Notifier
 *
 * Utility class for sending popup notifications and switching dashboard tabs
 * via the Elastic and QFRC dashboard systems.
 *
 * Features:
 * - Quick send error, warning, info notifications to both dashboards.
 * - Full customization support.
 * - Switch dashboard tabs by name or index.
 *
 * Notes:
 * - Ensure Elastic and QFRC dashboards are running and connected via NetworkTables.
 * 
 * @author CD
 * @since 2025 FRC Off Season
 */
public class Notifier {

  /**
   * Sends an error notification to both Elastic and QFRC dashboards.
   *
   * @param title       Notification title.
   * @param description Notification description.
   */
  public static void sendError(String title, String description) {
    sendCustom(3, title, description, 350, -1, 7000);
  }

  /**
   * Sends a warning notification to both Elastic and QFRC dashboards.
   *
   * @param title       Notification title.
   * @param description Notification description.
   */
  public static void sendWarning(String title, String description) {
    sendCustom(2, title, description, 350, -1, 7000);
  }

  /**
   * Sends an info notification to both Elastic and QFRC dashboards with default
   * display time of 7 seconds.
   *
   * @param title       Notification title.
   * @param description Notification description.
   */
  public static void sendInfo(String title, String description) {
    sendInfo(title, description, 7.0);
  }

  /**
   * Sends an info notification to both Elastic and QFRC dashboards with custom
   * display duration.
   *
   * @param title          Notification title.
   * @param description    Notification description.
   * @param displaySeconds Display duration in seconds.
   */
  public static void sendInfo(String title, String description, double displaySeconds) {
    sendCustom(1, title, description, 350, -1, (int) (displaySeconds * 1000));
  }

  /**
   * Sends a fully customized notification to both Elastic and QFRC dashboards.
   *
   * @param levelInt         Notification level integer (1 = Information, 2 = Warning, 3 = Critical).
   * @param title            Notification title.
   * @param description      Notification description.
   * @param width            Width of the notification widget.
   * @param height           Height of the notification widget (-1 for automatic).
   * @param displayTimeMillis Display time in milliseconds.
   */
  public static void sendCustom(
      int levelInt,
      String title,
      String description,
      double width,
      double height,
      int displayTimeMillis) {

    // Map int level to QFRC.ErrorLevel and Elastic.NotificationLevel
    ErrorLevel qfrcLevel = mapIntToQFRCLevel(levelInt);
    NotificationLevel elasticLevel = mapIntToElasticLevel(levelInt);

    // Create and send Elastic notification
    Elastic.Notification elasticNotification = new Elastic.Notification(elasticLevel, title, description)
        .withWidth(width)
        .withHeight(height)
        .withDisplayMilliseconds(displayTimeMillis);
    Elastic.sendNotification(elasticNotification);

    // Create and send QFRC notification
    QFRCLib.Notification qfrcNotification = new QFRCLib.Notification(qfrcLevel, title, description)
        .withWidth(width)
        .withHeight(height)
        .withDisplayMilliseconds(displayTimeMillis);
    QFRCLib.sendNotification(qfrcNotification);

    // Also report to QFRC error log as a message (not just pure errors)
    QFRCLib.reportError(qfrcLevel, title + ": " + description);
  }

  /**
   * Helper method to map integer (1-3) to QFRC.ErrorLevel.
   *
   * @param level Integer level (1 = Info, 2 = Warning, 3 = Critical).
   * @return Corresponding QFRC.ErrorLevel.
   */
  private static ErrorLevel mapIntToQFRCLevel(int level) {
    switch (level) {
      case 3:
        return ErrorLevel.Critical;
      case 2:
        return ErrorLevel.Warning;
      case 1:
      default:
        return ErrorLevel.Information;
    }
  }

  /**
   * Helper method to map integer (1-3) to Elastic.NotificationLevel.
   *
   * @param level Integer level (1 = Info, 2 = Warning, 3 = Error).
   * @return Corresponding Elastic.NotificationLevel.
   */
  private static NotificationLevel mapIntToElasticLevel(int level) {
    switch (level) {
      case 3:
        return NotificationLevel.ERROR;
      case 2:
        return NotificationLevel.WARNING;
      case 1:
      default:
        return NotificationLevel.INFO;
    }
  }

  /**
   * Switches the dashboard to the specified tab by name.
   *
   * @param tabName The tab to switch to.
   */
  public static void selectTab(String tabName) {
    Elastic.selectTab(tabName);
    QFRCLib.setTab(tabName);
  }

  /**
   * Switches the dashboard to the specified tab by index.
   *
   * @param tabIndex The tab index to switch to.
   */
  public static void selectTab(int tabIndex) {
    Elastic.selectTab(tabIndex);
    // QFRC may not support tab index selection directly.
  }
}
