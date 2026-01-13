package team7111.lib.pathfinding;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PosePlanning {

    // Member variable accessible by all methods
    private List<Pose2d> avoidPoses = new ArrayList<>();

    // Populate avoidPoses from field elements
    public void setAvoidPose(FieldElement[] fieldElements) {
        avoidPoses.clear(); // clear old obstacles if needed
        
        for (FieldElement element : fieldElements) {
            Pose2d[] poses;
            if (element.returnCorners() == null) {
                poses = element.returnCirclePose();
            } else {
                poses = element.returnCorners();
            }
            if (poses != null) {
                Collections.addAll(avoidPoses, poses);
            }
        }
    }

    public boolean isBlocked(Translation2d pos, double safetyRadius) {
        for (Pose2d obstacle : avoidPoses) {
            if (pos.getDistance(obstacle.getTranslation()) < safetyRadius) {
                return true;

            }
        }
        return false;
    }
}
