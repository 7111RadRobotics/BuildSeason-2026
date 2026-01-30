package team7111.lib.pathfinding;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import team7111.robot.subsystems.SwerveSubsystem;

public class Pathfinding {
    SwerveSubsystem swerve;
    Pose2d robotPose;
    Pose2d startPose;
    Translation2d gridPosition;
    double startPoseX;
    double startPoseY;
    public Pathfinding(SwerveSubsystem swerve) {
        this.swerve = swerve;
        startPose = swerve.getPose();
        robotPose = swerve.getPose();
        gridPosition = new Translation2d(Math.round(robotPose.getX() / gridSize), Math.round(robotPose.getY() / gridSize));
        startPoseX = startPose.getX();
        startPoseY = startPose.getY();
    }
    Translation2d wayPos = null;
    int[][] directions = {
        {1, 0},
        {0, 1},
        {-1, 0},
        {0, -1},
        {1, 1},
        {-1, -1},
        {1, -1},
        {-1, 1},
    };
    double gridSize = 0.25;
    double robotRadius = 0.5;
    double currentDistance;
    double startDistance;
    double pathWeight;
    double safety = robotRadius + 0.1;
    double minScore = -1;
    double minScoreX;
    double minScoreY;
    int dx;
    int dy;

    private List<Pose2d> avoidPoses = new ArrayList<>();

    private void setAvoidPose(FieldElement fieldElements) {
            Pose2d[] poses;
            if (fieldElements.returnCorners() == null) {
                poses = fieldElements.returnCirclePose();
            } else {
                poses = fieldElements.returnCorners();
            }
            if (poses != null) {
                Collections.addAll(avoidPoses, poses);
            }
    }

    public void avoidFieldElements(Waypoint waypoint, FieldElement fieldElements) {
        wayPos = new Translation2d(waypoint.getPose().getX(), waypoint.getPose().getY());

        setAvoidPose(fieldElements);

        for (int[] dir : directions) {
            Translation2d neighbor = new Translation2d(gridPosition.getX() + (dir[0] * 0.25), gridPosition.getY() + (dir[1] * 0.25));
            dx = dir[0];
            dy = dir[1];

            for (Pose2d obstacle : avoidPoses) {
                currentDistance = Math.sqrt(
                Math.pow(gridPosition.getX() + (dir[0] * 0.25) - wayPos.getX(), 2) +
                Math.pow(gridPosition.getY() + (dir[1] * 0.25) - wayPos.getY(), 2)
                );
                startDistance = Math.sqrt(
                    Math.pow(gridPosition.getX() - startPose.getX(), 2) +
                    Math.pow(gridPosition.getY() - startPose.getY(), 2)
                );
                pathWeight = currentDistance + startDistance;
                if (neighbor.getDistance(obstacle.getTranslation()) > safety) {
                    if (minScore == -1) {
                      minScore = pathWeight;
                    } else if (minScore >= pathWeight) {
                      minScore = pathWeight;
                      minScoreX = dx;
                      minScoreY = dy;
                    }
                }
            }
        }
        startPoseX = startPoseX + (dx*0.25);
        startPoseY = startPoseY + (dx*0.25);
        robotPose = new Pose2d(robotPose.getTranslation().getX()+(dx*0.25), robotPose.getTranslation().getY()+(dy*0.25), robotPose.getRotation());
        System.out.println("Min Node: (" + dx + ", " + dy + "), " + minScore);
        System.out.println("Current Pos:" + startPoseX + "," + startPoseY);
    }
}