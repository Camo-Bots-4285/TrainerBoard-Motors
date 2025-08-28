package frc.robot.HumanInterface;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static edu.wpi.first.units.Units.*;

import frc.robot.lib.Communication.DigitalButtons.ButtonGroup;
import frc.robot.subsystems.FlyWheelSubsystem;


/**
 * Tunning
 * 
 * Provides a set of digital buttons (via NetworkTables) used to trigger SysID 
 * routines for motor tuning. Supports both quasistatic and dynamic tests in 
 * forward and reverse directions.
 * 
 * This class is meant for testing and characterization only — not for use during competition.
 * 
 * Example usage includes tuning PID/FF constants or analyzing system response for feedforward.
 */
public class Tunning {

    // ========== Subsystems & NetworkTables ==========
    private final FlyWheelSubsystem m_flyWheel;
    private final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    public final NetworkTable table = networkTableInstance.getTable("TunningDigitalButtons");

    // Button labels for the tuning GUI
    private final String[] SysID_Buttons_Names = {
        "quasistatic_forward",
        "quasistatic_reverse",
        "dynamic_forward",
        "dynamic_reverse"
    };

    // Group of buttons used to trigger sysID commands
    public final ButtonGroup SysID_ButtonGroup = new ButtonGroup(table, SysID_Buttons_Names);

    /**
     * Creates a new Tunning interface.
     * Pass the target subsystem you want to run SysID tests on.
     *
     * @param m_flyWheel The Neo_Single subsystem to test.
     */
    public Tunning(FlyWheelSubsystem m_flyWheel) {
        this.m_flyWheel = m_flyWheel;
    }

    /**
     * Binds the SysID buttons to run the corresponding tuning routines.
     * Each trigger will run a specific direction and type of characterization.
     */
    public void TunnerBindings() {
        //Maps the following sysID to the buttons note you should only run one sysID at a time
        designatedSysIDRoutine(getSysIdRoutine(1, 7, 10));
    }

    /**
     * designatedSysIDRoutine
     * 
     * <p>Maps the given sysId to the digital buttons
     * @param routine
     */
    private void designatedSysIDRoutine(SysIdRoutine routine){
       
        // Quasistatic forward test
        Trigger btn_quasistatic_forward = SysID_ButtonGroup.getDashboardEntryAsTrigger(0);
        btn_quasistatic_forward.whileTrue(routine.quasistatic(Direction.kForward));

        // Quasistatic reverse test
        Trigger btn_quasistatic_reverse = SysID_ButtonGroup.getDashboardEntryAsTrigger(1);
        btn_quasistatic_reverse.whileTrue(routine.quasistatic(Direction.kReverse));

         // Dynamic forward test
        Trigger btn_dynamic_forward = SysID_ButtonGroup.getDashboardEntryAsTrigger(2);
        btn_dynamic_forward.whileTrue(routine.dynamic(Direction.kForward));

        // Dynamic reverse test
        Trigger btn_dynamic_reverse = SysID_ButtonGroup.getDashboardEntryAsTrigger(3);
        btn_dynamic_reverse.whileTrue(routine.dynamic(Direction.kReverse));
        
    }


    public SysIdRoutine getSysIdRoutine(double rampRate, double voltage,double time) {
        //new SysIdRoutine.Conf
              /* SysId routine for characterizing a motion. This is used to find PID gains for a motor. */
                   SysIdRoutine sysIdRoutine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                        Volts.of(rampRate).per(Second),        // Use default ramp rate (1 V/s)
                        Volts.of(voltage), // Reduce dynamic step voltage to 4 V to prevent brownout
                        Seconds.of(time),        // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> Logger.recordOutput("SysId_State", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                        output -> 
                            /* output is actually radians per second, but SysId only supports "volts" */
                        m_flyWheel.FlyWheel_Motor.setVoltage(output.in(Volts)),
                        null,
                        m_flyWheel
                    )
                );

        return sysIdRoutine;
    }
}
