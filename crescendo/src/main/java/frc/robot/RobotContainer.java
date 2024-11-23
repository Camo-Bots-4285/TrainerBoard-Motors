// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.Intake.FloorFeederTest;
import frc.robot.commands.SingleMotorAKCommands.*;
//import frc.robot.commands.*;

import frc.robot.subsystems.*;
import frc.robot.subsystems.SingleMotorAK.*;
import frc.robot.subsystems.TwoMotorAK.*;
import frc.robot.Constants.*;

import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // do not uncomment public static ArmPivotSubsystem m_armPivotSubsystem = new ArmPivotSubsystem(m_swerveBase);

  public static SingleMotor m_single_motor = new SingleMotor(null);
  public static TwoMotor m_two_motor = new TwoMotor(null);

  
 
 

  //Defines all mChooser that will display in suffle board
  private SendableChooser<String> mChooser;
  private SendableChooser<String> mChooser1;
  private SendableChooser<String> mChooser2;
  private SendableChooser<String> mChooser3;
  private SendableChooser<String> mChooser4;
  private SendableChooser<String> mChooser5;
  private SendableChooser<String> mChooser6;
  

  // public PowerDistributionPanel newPower = new PowerDistributionPanel(0);
  // public ClimberSubsystem m_climber = new ClimberSubsystem();
  // private ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /* Human interfaces */
  private final Joystick driverJoystick;
  private final Joystick streamdeck;
  
  //Defines all buttons that will be used to control the robot
  // private JoystickButton btn_arm_pivot_down;

  private JoystickButton btn_single_motor_spin;
  private JoystickButton btn_single_motor_pos;


  private DoubleSupplier limit;
  private DoubleSupplier stopRotation;
  private DoubleSupplier stopMainualDriving;
  private BiFunction<Double, Double, Double> Clamp;
  
  public static PIDController angleController;

  public static boolean Camera1_InAuto;
  public static boolean Camera2_InAuto;
  public static boolean Camera3_InAuto;
  public static boolean Camera4_InAuto;
  public static boolean Camera5_InAuto;
  

//  public static boolean isRed= true;
 
  /* Subsystems */
  // to bring back arm pivot


  
  /* Parent Class */
  private final Robot m_robot;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = 
  new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {
    m_robot = robot;

    //Defines what port of the compter each controler will be located in
    driverJoystick = new Joystick(0);
    streamdeck = new Joystick(1);


    //Put auto in here and they will show up in smart dash board do no forget to select auto before match
    mChooser = new SendableChooser<>();
    SmartDashboard.putData("Auto Choices",  mChooser);
    mChooser.setDefaultOption("Default Auto", "StraitLineTest");
    mChooser.addOption("6 Piece", "C-N3-Shoot6-N7-Shoot6");
    mChooser.addOption("Test Aim", "AlignShooterTest");
    mChooser.addOption("A-N1", "A-N1");
    mChooser.addOption("Hellos", "Hellos");
    mChooser.addOption("B-Shoot2-N2-Shoot5", "B-Shoot2-N2-Shoot5");
    mChooser.addOption("StraitLineTest", "StraitLineTest");
    mChooser.addOption("Note Stop Test", "Note Stop Test");
    mChooser.addOption("NotePickupTest", "NotePickupTest");
    mChooser.addOption("NotePickupTest2", "NotePickupTest2");
    mChooser.addOption("103 Auto", "103 Auto");
    mChooser.addOption("RotationTest", "RotationTest");
    

    //Used to turn Cameras on and off in auto
    mChooser1 = new SendableChooser<>();
    mChooser1.setDefaultOption("Yes", "Yes");
    mChooser1.addOption("No", "No");
    SmartDashboard.putData("Camera-1 in Auto" ,  mChooser1);

    mChooser2 = new SendableChooser<>();
    mChooser2.setDefaultOption("Yes", "Yes");
    mChooser2.addOption("No", "No");
    SmartDashboard.putData("Camera-2 in Auto" ,  mChooser2);

    mChooser3 = new SendableChooser<>();
    mChooser3.setDefaultOption("Yes", "Yes");
    mChooser3.addOption("No", "No");
    SmartDashboard.putData("Camera-3 in Auto" ,  mChooser3);

    mChooser4 = new SendableChooser<>();
    mChooser4.setDefaultOption("Yes", "Yes");
    mChooser4.addOption("No", "No");
    SmartDashboard.putData("Camera-4 in Auto" ,  mChooser4);

    mChooser5 = new SendableChooser<>();
    mChooser5.setDefaultOption("Yes", "Yes");
    mChooser5.addOption("No", "No");
    SmartDashboard.putData("Camera-5 in Auto" ,  mChooser5);

    // mChooser6 = new SendableChooser<>();
    // mChooser6.addOption(, null);
    // mChooser6.addOption("No", "No");
    // SmartDashboard.putData("Camera-1 in Auto" ,  mChooser1);

    

    //Used to make isRed true or false to inverts what side the robot is on
    // mChooser6 = new SendableChooser<>();
    // mChooser6.setDefaultOption("Red", "Red");
    // mChooser6.addOption("Blue", "Blue");
    // SmartDashboard.putData("Aliance Color",  mChooser6);

 
    // Controles rotaion Whne Auto Targeting
     angleController = new PIDController(1.0, 0.0, 0.0);
     angleController.enableContinuousInput(-Math.PI, Math.PI);
    
         //Added by Spencer to ramp power by lever
    //limit = () -> 0.55 - 0.45 * driverJoystick.getRawAxis(SwerveConstants.sliderAxis);
    /* maps sliderAxis to be between 0.1 and 1.0 */
   // Clamp  = (val, lim) -> (Math.abs(val) < lim) ? val : Math.copySign(lim, val);

   //Old controler interface that used Spencer clamp which had now been moved to control max speed thought TeleOp Swerve
     // m_swerveBase = new SwerveBase();
    // m_swerveBase.setDefaultCommand(
    //     new TeleopSwerve(
    //         m_swerveBase,
    //         () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis) , limit.getAsDouble()),
    //         () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
    //         () -> -Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.rotationAxis) ,limit.getAsDouble() * stopRotation.getAsDouble()),
    //         () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
            
    //     ));

    //Stops the robot from reciving any rotation command
   stopRotation = () -> driverJoystick.getRawButton(9) ? 0.0 : 1.0;
    //New driver interface without clamp and new lever ramp range from 20%-100% commanded max power
  
  
   
  }


  public Joystick getJoystick() {
    return driverJoystick;
  }

  //
  public Joystick getstreamdeck() {
    return streamdeck;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // btn_example = new Trigger(m_exampleSubsystem::exampleCondition)
    // btn_example.onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());



    //The following commands should be transfered year to you

   
  
    btn_single_motor_spin = new JoystickButton(streamdeck, 13);
    btn_single_motor_spin.whileTrue(new SingleMotorSpin(m_single_motor));
 
    btn_single_motor_pos = new JoystickButton(streamdeck, 14);
    btn_single_motor_pos.whileTrue(new SingleMotorPos(m_single_motor));

   
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return AutoBuilder.buildAuto(mChooser.getSelected());//mChooser.getSelected() will get the auto selected from smart dashboard
    //if you want hard coded auto do "AutoName"
  //   Command autonomousCommand = new AutoTest(
  //   m_shooter,
  //   m_ArmPivotSubsystem,
  //   m_intake,
  //   m_shooterFeeder,
  //   m_swerveBase,
  //   m_lineBreak
  //  );

    //return autonomousCommand;
  }



  
  //Take what mChooser says and makes isRed true or false only work beacuse method is called in robot(telop perodic)
  public void SmartDashboardtoCommands() {
  // if (mChooser6.getSelected() == "Blue") {
  //  isRed = false;
  // }
  // if (mChooser6.getSelected() == "Red"){
  //  isRed = true;
  //   }



  if (mChooser1.getSelected() == "No") {
    Camera1_InAuto = false;
  }
  if (mChooser1.getSelected() == "Yes"){
   Camera1_InAuto = true;
  }
  
  if (mChooser2.getSelected() == "No") {
    Camera2_InAuto = false;
  }
  if (mChooser2.getSelected() == "Yes"){
    Camera2_InAuto = true;
  }
  
  if (mChooser3.getSelected() == "No") {
    Camera3_InAuto = false;
  }
  if (mChooser3.getSelected() == "Yes"){
    Camera3_InAuto = true;
  }

  if (mChooser4.getSelected() == "No") {
    Camera4_InAuto = false;
  }
  if (mChooser4.getSelected() == "Yes"){
   Camera4_InAuto = true;
  }
  
  if (mChooser5.getSelected() == "No") {
   Camera5_InAuto = false;
  }
  if (mChooser5.getSelected() == "Yes"){
    Camera5_InAuto = true;
  }
  
  }
  
}