package team7111.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class is an example to how a subsystem looks and functions.
 * The name of the file and class should be what it controls
 */
public class Example extends SubsystemBase {
    
    /**
     * The enum that holds the values of the subsystem's states.
     * It's name should be the subsystem's followed by "State"
     */
    public enum ExampleState {
        example,
        otherExample,
        thirdExample,
    }

    private ExampleState state = ExampleState.example;

    public Example() {}

    public void periodic(){
        manageState();
    }

    public void simulationPeriodic(){}

    private void manageState(){
        switch(state){
            case example:
                example();
                break;
            case otherExample:
                otherExample();
                break;
            case thirdExample:
                thirdExample();
                break;
            default:
                break;
        }
    }

    private void example(){
        System.out.println("Runs code for the example state");
    }

    private void otherExample(){
        System.out.println("Runs code for the otherExample state");
    }

    private void thirdExample(){
        System.out.println("Runs code for the thirdExample state");
    }

    public void setState(ExampleState state){
        this.state = state;
    }

    public ExampleState getState(){
        return state;
    }
}
