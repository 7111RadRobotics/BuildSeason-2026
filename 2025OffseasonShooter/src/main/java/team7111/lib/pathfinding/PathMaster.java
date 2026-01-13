package team7111.lib.pathfinding;

import java.util.ArrayList;
import java.lang.reflect.Array;
import java.util.Collections;
import java.util.Arrays;
import java.util.function.Supplier;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import team7111.robot.subsystems.PathSubsystem;
import team7111.robot.subsystems.SwerveSubsystem;
import team7111.lib.pathfinding.PosePlanning;

public class PathMaster {
    private PIDController xPID;
    private PIDController yPID;
    private PIDController rotPID;

    private Supplier<Pose2d> suppliedPose;
    private Supplier<Rotation2d> gyroYaw;

    private double xCalculation;
    private double yCalculation;
    private double rotCalculation;

    private double invertedX = 1.0;
    private double invertedY = 1.0;
    private double invertedRot = 1.0;
    private double invertedGyro = 1.0;

    //Houses field elements
    private FieldElement fieldElements[];

    private boolean shouldFlipPath = false;
    private boolean isPathMirrored = false;
    private boolean fieldRelative = false;

    private boolean avoidFieldElements = false;
    private Translation2d initialPosition = null;
    private Translation2d currentPosition = null;
    private double G = 0;
    private boolean pathfinding = false;

    public PathMaster(Supplier<Pose2d> suppliedPose, Supplier<Rotation2d> gyroYaw){

        xPID = new PIDController(1, 0, 0);
        yPID = new PIDController(1, 0, 0);

        this.rotPID = new PIDController(1, 0, 0);
        rotPID.enableContinuousInput(-180, 180);
        this.suppliedPose = suppliedPose;
        this.gyroYaw = gyroYaw;
    }
    
    /**
     * Determines if the path should be flipped or not.
     * @param flipField -Which team it is on. False = alliance 1, true = alliance 2
     * @param isMirrored -Determines if the path should be flipped. If false, it will not flip.
     */
    public void useAllianceFlipping(boolean flipField, boolean isMirrored){
        shouldFlipPath = flipField;
        isPathMirrored = isMirrored;
    }
    
    public void useFieldRelative(boolean isFieldRelative){
        fieldRelative = isFieldRelative;
    }

    /**
     * Sets the translation pid for the path
     */
    public void setTranslationPID(double P, double I, double D){
        xPID.setPID(P, I, D);
        yPID.setPID(P, I, D);
    }

    /**
     * Sets the rotation pid for the path
     */
    public void setRotationPID(double P, double I, double D){
        rotPID.setPID(P, I, D);
    }

    /**
     * Sets the field element map to set where the objects are
     */
    public void setFieldElementMap(FieldElement[] fieldElementArray){
        fieldElements = fieldElementArray;
    }

    /**
     * Sets the inversions for the robot
     */
    public void setInversions(boolean invertX, boolean invertY, boolean invertRot, boolean invertGyro){
        invertedX = 1.0;
        invertedY = 1.0;
        invertedRot = 1.0;
        invertedGyro = 1.0;

        if (invertX){
            invertedX = -1.0;
        }

        if (invertY){
            invertedY = -1.0;
        }

        if (invertRot){
            invertedRot = -1.0;
        }

        if (invertGyro){
            invertedGyro = -1.0;
        }
    }

    
    void useBrokenPathFinding(boolean avoidFieldElements){
        this.avoidFieldElements = avoidFieldElements;
        pathfinding = true;
        initialPosition = suppliedPose.get().getTranslation();
    }

    
    /**
     * Equivelant to a "Reset" command, it applies all setup methods and initilizes the path
     */
    public void initializePath(Path path){
        path.setPoseSupplier(suppliedPose);
        path.setSpeedSuppliers(()-> xCalculation, ()-> yCalculation, ()-> rotCalculation);
        path.flipPath(shouldFlipPath, isPathMirrored);
        path.avoidFieldElements(avoidFieldElements, fieldElements);
        path.initialize();
    }

    Translation2d waypointPos = null;
    Translation2d currentPos = null;
    double H = 0;
    double fieldLength = 0;
    double fieldWidth = 0;
    final double gridSize = 0.25;
    final double robotRadius = 0.4;
    private PosePlanning planner = new PosePlanning();
    double[][] directions = {
        {1,0},
        {0,1},
        {-1,0},
        {0,-1},
        {1,1},
        {-1,1},
        {1,-1},
        {-1,-1},
    };
    double currentX = 0;
    double currentY = 0;
    double F = 0;

    /**
     * Runs the path's periodic and calculates the speed suppliers for x, y, and rotation.
     */
    public void periodic(Path path){
        path.periodic();
        xCalculation = xPID.calculate(suppliedPose.get().getX(), path.getCurrentWaypoint().getPose().getX()) * invertedX;
        yCalculation = yPID.calculate(suppliedPose.get().getY(), path.getCurrentWaypoint().getPose().getY()) * invertedY;
        rotCalculation = rotPID.calculate(
                            suppliedPose.get().getRotation().getDegrees(), 
                            path.getCurrentWaypoint().getPose().getRotation().getDegrees()) * invertedRot;
        if (pathfinding) {
            waypointPos = path.getCurrentWaypoint().getPose().getTranslation();
            currentPos = suppliedPose.get().getTranslation();
            currentX = currentPos.getX();
            currentY = currentPos.getY();
            H = Math.hypot(
                waypointPos.getX() - currentPos.getX(),
                waypointPos.getY() - currentPos.getY()
            );
            currentPosition = suppliedPose.get().getTranslation();
            G = Math.sqrt(
                Math.pow(currentPosition.getX() - initialPosition.getX(), 2) +
                Math.pow(currentPosition.getY() - initialPosition.getY(), 2)
            );
            F = H + G;
            for (double[] dir : directions) {
                double nx = currentX + dir[0];
                double ny = currentY + dir[1];
            
                double neighborX = nx * gridSize;
                double neighborY = ny * gridSize;

                Translation2d neighbor = new Translation2d(neighborX, neighborY);

                if (planner.isBlocked(neighbor, robotRadius)) {
                    continue;
                }
            }
        }
    }

    /**
     * Returns a ChassisSpeeds object for handing to the swerve subsystem
     */
    public ChassisSpeeds getPathSpeeds(Path path, boolean avoidFieldElements, boolean fieldRelative){
        
        ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(path.getTranslationXSpeed(), path.getTranslationYSpeed(), path.getRotationSpeed(), gyroYaw.get().times(invertedGyro))
            : new ChassisSpeeds(path.getTranslationXSpeed(), path.getTranslationYSpeed(), path.getRotationSpeed());

        return chassisSpeeds;

    }
}
