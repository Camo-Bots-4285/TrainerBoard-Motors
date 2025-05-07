package frc.robot.util.Communication;

import frc.robot.util.Communication.Elastic.Notification;
import frc.robot.util.Communication.Elastic.Notification.NotificationLevel;

/**
 * ElasticNotifier
 *
 * Description:
 * This utility class provides a clean and concise interface for sending popup notifications
 * and switching dashboard tabs via the Elastic dashboard system. It wraps around the
 * {@link Elastic} class and simplifies common tasks such as sending error, warning,
 * info, and custom messages from robot code.
 *
 * Features:
 * - Send error, warning, or info notifications with minimal setup.
 * - Fully customize notifications (size, duration, severity).
 * - Programmatically switch between dashboard tabs by name or index.
 *
 * Notes:
 * - No explicit initialization is required; Elastic handles publisher setup internally.
 * - Ensure the Elastic dashboard is running and connected via NetworkTables.
 *
 * TODO: Test by trying to sent a notificaiton to elastic
 */
public class ElasticNotifier {

  /**
   * Sends a quick error-level notification to the dashboard with the specified title and description.
   *
   * @param title       the title of the notification (e.g., "Error Detected")
   * @param description the description text explaining the error
   */
  public static void sendError(String title, String description) {
    Notification notification = new Notification(NotificationLevel.ERROR, title, description);
    Elastic.sendNotification(notification);
  }

  /**
   * Sends a quick warning-level notification to the dashboard with the specified title and description.
   *
   * @param title       the title of the notification (e.g., "Low Battery")
   * @param description the description text explaining the warning
   */
  public static void sendWarning(String title, String description) {
    Notification notification = new Notification(NotificationLevel.WARNING, title, description);
    Elastic.sendNotification(notification);
  }

  /**
   * Sends an info-level notification with a custom display duration.
   *
   * @param title          the title of the notification (e.g., "Match Started")
   * @param description    the description text providing more details
   * @param displaySeconds how long the message should stay visible (in seconds)
   */
  public static void sendInfo(String title, String description, double displaySeconds) {
    Notification notification = new Notification()
        .withLevel(NotificationLevel.INFO)
        .withTitle(title)
        .withDescription(description)
        .withDisplaySeconds(displaySeconds);
    Elastic.sendNotification(notification);
  }

  /**
   * Sends a fully customized notification with all parameters specified.
   *
   * @param level             the severity level (INFO, WARNING, ERROR)
   * @param title             the title of the notification
   * @param description       the main body text of the notification
   * @param width             the width of the notification window (pixels)
   * @param height            the height of the notification window (pixels, -1 = auto)
   * @param displayTimeMillis how long to display the message (in milliseconds)
   */
  public static void sendCustom(
      NotificationLevel level,
      String title,
      String description,
      double width,
      double height,
      int displayTimeMillis) {

    Notification notification = new Notification(level, title, description)
        .withWidth(width)
        .withHeight(height)
        .withDisplayMilliseconds(displayTimeMillis);
    Elastic.sendNotification(notification);
  }

  /**
   * Switches the dashboard to the specified tab by name.
   * If the tab does not exist, nothing will happen.
   *
   * @param tabName the name of the tab to switch to (e.g., "Match Info")
   */
  public static void selectTab(String tabName) {
    Elastic.selectTab(tabName);
  }

  /**
   * Switches the dashboard to the specified tab by index.
   * Tab index starts at 0 for the first tab.
   *
   * @param tabIndex the index of the tab to switch to (e.g., 2 for third tab)
   */
  public static void selectTab(int tabIndex) {
    Elastic.selectTab(tabIndex);
  }
}
