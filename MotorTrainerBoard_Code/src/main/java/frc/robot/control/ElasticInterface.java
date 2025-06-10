package frc.robot.control;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * ElasticInterface
 * <p>
 * Description:
 * This class will hold all button and SendableChooser that are to be displayed in Elastic 
 * <p>
 * Notes: This will change from year to year
 * <p>
 * TODO: Add more button and map them to commands 
 */
public class ElasticInterface {

    public static SendableChooser<String> mChooser_Robot;
    public static SendableChooser<String> mChooser_Battery;
    public static SendableChooser<String> mChooser_Auto;

/**
 * ElassticInterface
 * <p>
 * Description:
 * Hold Senable chooser to be run on intation 
 * <p>
 * Notes: This will change from year to year
 * <p>
 * TODO: Add more sendable choosers as needed 
 */
    public ElasticInterface(){

    mChooser_Robot = new SendableChooser<>();
    SmartDashboard.putData("Current Robot", mChooser_Robot);
    mChooser_Robot.setDefaultOption("Comp_Robot", "Compatition_Robot");
    mChooser_Robot.addOption("Robot_2", "Robot_2");
    mChooser_Robot.addOption("Robot_3", "Robot_3");

    mChooser_Battery = new SendableChooser<>();
    SmartDashboard.putData("Current Battery", mChooser_Battery);
    mChooser_Battery.setDefaultOption("Unselected", "Someone Forgot Something Sorry :(");
    mChooser_Battery.addOption("3_Orange_2022", "3_Orange_2022");
    mChooser_Battery.addOption("__Yellow_2022", "__Yellow_2022");
    mChooser_Battery.addOption("6_Red_2023", "6_Red_2023");

    mChooser_Auto = new SendableChooser<>();
    SmartDashboard.putData("Current Auto", mChooser_Auto);
    mChooser_Auto.setDefaultOption("Unselected", "Someone Forgot Something Sorry :(");
    mChooser_Auto.addOption("Auto_2", "Auto_2");
    mChooser_Auto.addOption("Auto_3", "Auto_3");
    }
}
