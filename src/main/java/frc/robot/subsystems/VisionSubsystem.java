package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Utils.RollingAverage;

public class VisionSubsystem extends SubsystemBase {

    public static PhotonCamera photonCamera;
    public static AprilTagFieldLayout layout;

    public static PhotonPoseEstimator poseEstimator;

    public static RollingAverage distanceAverage = new RollingAverage(5);

    public VisionSubsystem() {

        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        Transform3d transform3d = new Transform3d(new Translation3d(0.17145, 0.5588, -0.2159), new Rotation3d(0, 0, 0));

        photonCamera = new PhotonCamera("ArduCam");
        poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, transform3d);
    }

    public double getDistance(double height){
        double x = height;
        double result = -0.001999989829743508*x*x*x + 0.29421208629445184*x*x + -14.42334703010702*x + 281.74836724325877; //CURVE:distance,08:56,03/16
        return result;
    }

    @Override
    public void periodic() {
            var result = photonCamera.getLatestResult();

            if(result.hasTargets() && Math.abs(result.getBestTarget().getYaw()) > 20){
                var best = result.getBestTarget();

         

                //if(best.getPoseAmbiguity() < 0.9)
                //return;
                
                poseEstimator.setReferencePose(RobotContainer.driveTrain.getPose());
                var estimated = poseEstimator.update();
                
                

                if(!estimated.isPresent())
                return;

                var newPose = estimated.get();

                RobotContainer.driveTrain.setPose(newPose.estimatedPose.toPose2d());
            }
            /* 

            if(result.hasTargets() && Math.abs(result.getBestTarget().getYaw()) < 20){

                var bestTarget = result.getBestTarget();

                if(bestTarget.getFiducialId() == 4)
                return;

                

                double yaw = 180 + bestTarget.getYaw() - RobotContainer.driveTrain.getGyroDegrees();

                TargetCorner bottomCorner = bestTarget.getDetectedCorners().get(0);
                TargetCorner topCorner = bestTarget.getDetectedCorners().get(3);

                double height = bottomCorner.y - topCorner.y;
                double distance = Units.inchesToMeters(getDistance(height));
                distanceAverage.put(distance);
                SmartDashboard.putNumber("Height", height);
                SmartDashboard.putNumber("Distance", distance);

                double y = Math.sin(Math.toRadians(yaw)) * distanceAverage.get();
                double x = Math.cos(Math.toRadians(yaw)) * distanceAverage.get();
                var tagPose = layout.getTagPose(bestTarget.getFiducialId());
                double robotX = tagPose.get().getX() + -x;
                double robotY = tagPose.get().getY() + y;
                
                RobotContainer.driveTrain.setPose(robotX, robotY, 0);
                return;   
            }
            distanceAverage.clear();
            */
        }
}