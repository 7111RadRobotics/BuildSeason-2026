package team7111.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.lib.pathfinding.Path;
import team7111.lib.pathfinding.Waypoint;
import team7111.lib.pathfinding.WaypointConstraints;
import team7111.robot.subsystems.SuperStructure.SuperState;
import team7111.robot.utils.AutoAction;

public class Autonomous extends SubsystemBase {

    private WaypointConstraints translationConstraints = new WaypointConstraints(5, 0, 0.001);
    private WaypointConstraints rotationConstraints = new WaypointConstraints(720, 0, 1);
    
    public enum Autos {
        shootPreload,
        forwardTest,
        rotateTest,
    }

    public enum Paths {
        forward,
        rotate90
    }

    public Autonomous(){

    }

    public void periodic(){}

    public void simulationPeriodic(){}

    public List<AutoAction> getAutonomous(Autos autoName){
        ArrayList<AutoAction> auto = new ArrayList<>();
        // define each autonomous using a switch statement.
        // each auto is an array of "AutoAction's"
        switch (autoName) {
            case forwardTest:
                auto.add(new AutoAction(getPath(Paths.forward)));
                break;
            case rotateTest:
                auto.add(new AutoAction(getPath(Paths.rotate90)));
                break;
        
            default:

                break;
        }
        return auto;
    }

    public Path getPath(Paths path){
        List<Waypoint> waypoints = new ArrayList<>();
        // define Path object for each Paths enum using a switch statement
        switch (path) {
            case forward:
                waypoints.add(new Waypoint(new Pose2d(1, 0, Rotation2d.fromDegrees(0)), translationConstraints, rotationConstraints));
                break;
            case rotate90:
                waypoints.add(new Waypoint(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), translationConstraints, rotationConstraints));
                break;
        }
        return new Path(waypoints);
    }
}
