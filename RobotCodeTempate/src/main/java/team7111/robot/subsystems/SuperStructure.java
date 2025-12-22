package team7111.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class handles the overall state of the robot.
 * It does this by defining various "SuperState" values and calling the methods assosiated with each one.
 * Each method uses logic to determine subsystem states, when to switch SuperStates, and what SuperState to go to next.
 * Logic used can be button inputs, subsystem states, or other subsystem conditions.
 */
public class SuperStructure extends SubsystemBase {
    /**
     * This enumeration contains the values or "states" which determine the subsystems.
     * The top 2 state names are placeholders. 
     */
    public enum SuperState {
        state1,
        state2,
        autonomousEnter,
        autonomous,
        autonomousExit,
    }

    private final ExampleSubsystem exampleSubsystem;

    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    private SuperState superState = SuperState.state1;

    private boolean inAuto = false;
    private int autoIndex = 0;

    /**
     * The constructor will take each subsystem as an argument and save them as objects in the class. 
     * @param subsystem represents a subsystem. 
     */
    public SuperStructure(ExampleSubsystem exampleSubsystem){
        this.exampleSubsystem = exampleSubsystem;
    }

    /**
     * This method runs every iteration (every 20ms). Stuff like state management and stateless logic are put in here.
     */
    public void periodic(){

    }

    /**
     * This method is run every iteration (20ms) only in simulation mode. Can be used to update simulated mechanisms or sensors.
     */
    public void simulationPeriodic(){

    }

    /**
     * This is called the state manager. It checks the value of the SuperState argument and calls the method associated with it.
     * @param state the SuperState value
     * @return the condition of the state determined by the state method.
     */
    private boolean manageSuperState(SuperState state){
        switch(state){
            case state1:
                return state1();
            case state2:
                return state2();
            case autonomous:
                return autonomous();
            case autonomousEnter:
                return autonomousEnter();
            case autonomousExit:
                return autonomousExit();
            default:
                return true;
        }
    }

    /**
     * Each of these methods (state1-autonomousExit) represents a defined state.
     * When called by the state manager, it will set the states of different subsystems.
     * @return true if the state is complete. The condition could represent mechanisms at a setpoint, a beambreak trigger, a timer, etc.
     * Mainly used for autonomous routines.
     */
    private boolean state1(){
        return true;
    }

    private boolean state2(){
        return true;
    }

    private boolean autonomousEnter(){
        autoIndex = 0;
        return true;
    }
    private boolean autonomous(){
        inAuto = true;

        return true;
    }
    private boolean autonomousExit(){
        return true;
    }

    public void setSuperState(SuperState state){
        if(inAuto)
            superState = state;
    }

    public SuperState getSuperState(){
        return superState;
    }
}
