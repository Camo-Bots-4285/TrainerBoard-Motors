package frc.robot.HumanInterface;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.CamoBots.Communication.DashBoard.DashBoardAddOns;


/**
 * ElasticDisplay
 * <p>
 * Holds buttons and SendableChoosers displayed on the Elastic dashboard.
 * This class is responsible for initializing and publishing UI elements.
 * <p>
 * TODO: Add more SendableChoosers or buttons as needed.
 */
public class ElasticDisplay extends SubsystemBase{

    //Defines senable choosers
    public static SendableChooser<String> mChooser_Robot = new SendableChooser<>();
    public static SendableChooser<String> mChooser_Battery = new SendableChooser<>();
    public static SendableChooser<String> mChooser_Auto= new SendableChooser<>();

   // private Telemetry m_Telemetry;

    /**
     * Constructor initializes SendableChoosers and dashboard widgets.
     *
     * @param m_Telemetry Telemetry subsystem data used for custom widgets.
     */
    public ElasticDisplay (/*Telemetry m_Telemetry*/) {
       // this.m_Telemetry=m_Telemetry;

        // Robot selection mchooser
        SmartDashboard.putData("Current Robot", mChooser_Robot);
        mChooser_Robot.setDefaultOption("Please Select Robot", "Someone Forgot Sorry :(");
        mChooser_Robot.addOption("Comp_Robot", "Competition_Robot");
        mChooser_Robot.addOption("Robot_2", "Robot_2");
        mChooser_Robot.addOption("Robot_3", "Robot_3");

        // Battery selection chooser
        SmartDashboard.putData("Current Battery", mChooser_Battery);
        mChooser_Battery.setDefaultOption("Please Select Battery", "Someone Forgot Sorry :(");
        mChooser_Battery.addOption("3_Orange_2022", "3_Orange_2022");
        mChooser_Battery.addOption("__Yellow_2022", "__Yellow_2022");
        mChooser_Battery.addOption("6_Red_2023", "6_Red_2023");

        // Autonomous mode chooser
        SmartDashboard.putData("Current Auto", mChooser_Auto);
        mChooser_Auto.setDefaultOption("Please Select Auto", "Default");
        mChooser_Auto.addOption("Custom", "Custom");
        mChooser_Auto.addOption("Square", "Square");
        mChooser_Auto.addOption("CurveTest", "CurveTest");

        // Custom Elastic widget to display swerve telemetry on dashboard
       // DashBoardAddOns.Swerve_display("Robot", m_Telemetry.m_moduleStatesArray, m_Telemetry.m_poseArray);
        DashBoardAddOns.FieldDisplay_int();
        DashBoardAddOns.FieldDisplay_clear();

        SmartDashboard.putData("Command Schudler", CommandScheduler.getInstance());

        //Add a photo as a camera stream updates every 5 sec
       //System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
       //new StaticImageStream("LogoImage", "/home/lvuser/deploy/logo.jpg", 640, 480);

    }

   @Override
   public void periodic() {
    //Auto updated the robot pose on the feild
    // DashBoardAddOns.FieldDisplay_UpdateRobotPose(m_Telemetry.m_poseArray);

    }
}
