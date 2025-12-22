package team7111.robot.utils;

import java.util.function.BooleanSupplier;

import team7111.lib.pathfinding.Path;
import team7111.robot.subsystems.SuperStructure.SuperState;

public class AutoAction {
    private Path path = null;
    private SuperState state = null;
    private boolean isPath = false;
    private boolean isState = false;
    private BooleanSupplier alternateCondition = null;
    private boolean hasAlternateCondition = false;

    public AutoAction(Path path){
        this.path = path;
        isPath = true;
    }
    /**
     * Constructs the object to contain a SuperState. Use AutoAction(Path) to contain a Path
     * @param state The object to contain as a SuperState
     */
    public AutoAction(SuperState state){
        this.state = state;
        isState = true;
    }

    public SuperState getAsState(){
        if(state != null){
            return state;
        }
        throw new NullPointerException("Instance of AutoAction does not contain SuperState!\nCheck constructor used for object instantiation");
    }

    public Path getAsPath(){
        if(path != null){
            return path;
        }
        throw new NullPointerException("Instance of AutoAction does not contain Path!\nCheck constructor used for object instantiation.");
    }

    public boolean isPath(){
        return isPath;
    }

    public boolean isState(){
        return isState;
    }

    public boolean hasAlternateCondition(){
        return hasAlternateCondition;
    }

    public boolean getAlternateCondition(){
        return alternateCondition.getAsBoolean();
    }

    public AutoAction withAlternateCondition(BooleanSupplier condition){
        this.alternateCondition = condition;
        hasAlternateCondition = true;
        return this;
    }
}
