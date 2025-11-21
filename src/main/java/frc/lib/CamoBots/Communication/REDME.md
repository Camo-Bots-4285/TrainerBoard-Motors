/**
 * Communications Utility Packages Summary
 *
 * This folder contains several utility classes to assist with communication and control 
 * in the FRC robot system. These classes interact with various FRC dashboard tools, Elastic 
 * and NetworkTables to provide efficient ways of handling button inputs, sending notifications,
 * and displaying real-time data.
 *
 * Classes:
 * 
 * 1. **ButtonGroup**:
 *    - This class allows management and interaction with a group of buttons in the NetworkTable.
 *    - Ensures that only one button is active at a time within a button group.
 *    - It supports fetching the active button index and name, and integrates with triggers for button presses.
 *    - Works with the NetworkTable to store and update button states.
 *
 * 2. **ButtonSingleToggle**:
 *    - Provides a utility for creating and interacting with a single button in the NetworkTable.
 *    - Allows for toggling the button state and using it in triggers to monitor button presses.
 *    - The button state is stored in a NetworkTable entry, making it easy to monitor and control.
 *
 * 3. **ElasticNotifier**:
 *    - Simplifies the process of sending notifications and switching tabs on the Elastic dashboard system.
 *    - Allows sending error, warning, info, and custom notifications to the dashboard.
 *    - Can switch between dashboard tabs either by name or index, providing a user-friendly interface for interaction.
 *
 * 4. **Elastic**:
 *    - Elastic is a modern, real-time dashboard interface for FRC robots, designed to replace 
 *      deprecated tools like Shuffleboard and SmartDashboard.
 *    - Offers real-time data visualization, customizable notifications, and tab management.
 *    - Works seamlessly with NetworkTables and is actively developed, ensuring teams have access 
 *      to new features and support.
 *    - Fully customizable and integrates easily into robot code for efficient data display and operator communication.
 * 
 * 5. **ElasticAddOns**:
 *    - This file should be frequently updated as new features and capabilities are added to the Elastic dashboard system.
 *    - Includes enhancements and additional functionalities that extend the capabilities of Elastic.
 *    - Allows teams to adapt to future updates and integrate them into their robot code effectively.
 *    - Current hold the method to show the swerve drive and the method to display color
 *
 * 6. **NetworkTables_Publishers**:
 *    - A utility for recording various types of data (such as doubles, integers, booleans, strings, poses, translations, and *         rotations) to NetworkTable entries.
 *    - Includes classes for recording individual values as well as arrays of values for real-time feedback.
 *    - Ideal for sending robot telemetry, sensor data, or position information to the dashboard for visualization.
 * 
 * Summary:
 * - This folder provides essential tools for improving robot-human communication during FRC matches.
 * - The utilities focus on using the new Elastic dashboard interface for real-time data visualization, 
 *   button management, and notifications.
 * - The classes interact with NetworkTables, providing flexibility and control over robot inputs and outputs.
 * - As Shuffleboard and SmartDashboard are deprecated and no longer actively supported, Elastic serves as the 
 *   future-proof solution for FRC dashboard needs.
 * - The RobotDataPublisher class enables us to publish a wide range of data types to the dashboard for real-time feedback and *        display.
 */
