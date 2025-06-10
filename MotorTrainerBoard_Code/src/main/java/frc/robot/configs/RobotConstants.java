package frc.robot.configs;


import frc.robot.Constants;

/**
 * RobotConstants
 * <p>
 * Description:
 * This class will pass though a differnt constants class depending on which robot is in use
 * <p>
 * Notes: Each method muse bedefine for each sub class so even if the robot you are coding does not need a certain method make sure to still add it
 * <p>
 * TODO: Add in more constants
 */
public interface RobotConstants {

  String ConstantsName();
  double[] Single_Neo_MotionProfile();


    /**
     * RobotConstants - Constructor
     * <p>
     * Description:
     * Passes thought the robot identity to be used
     */
  public static RobotConstants getRobotConstants(Constants.RobotIdentity robot) {
    switch (robot) {
      case Robot_1:
        return new Robot_1();
      case Robot_2:
        return new Robot_2();
      case Robot_3:
        return new Robot_3();
      default:

        // Something went wrong if this branch is reached, by default we will return our Comp Bot
        return new Robot_1();
    }
  }

}