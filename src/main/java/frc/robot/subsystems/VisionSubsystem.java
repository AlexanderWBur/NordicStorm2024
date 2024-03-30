package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.Utils.RollingAverage;

public class VisionSubsystem extends SubsystemBase {

    public static PhotonCamera photonCamera;
    public static PhotonCamera noteCamera;
    public static AprilTagFieldLayout layout;
    public boolean canSeeTarget = false;

    final double CAMERA_HEIGHT_METERS = 0.216;

    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(25);

    public static PhotonPoseEstimator poseEstimator;

    public static RollingAverage distanceAverage = new RollingAverage(1);

    List<PhotonTrackedTarget> notes = new ArrayList<PhotonTrackedTarget>();

    public PhotonTrackedTarget bestTarget = null;
    double camHeight = Units.inchesToMeters(27); //
    Transform3d transform3d = new Transform3d(new Translation3d(0.381, 0.1845, -0.2159),
            new Rotation3d(0, Math.toRadians(25), 0));

    public VisionSubsystem() {

        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        noteCamera = new PhotonCamera("NoteCam");
        photonCamera = new PhotonCamera("USB_2M_GS_camera");
        poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, photonCamera,
                transform3d);

    }

    public static double visToRealAngle(double ang) {

        double x = ang;
        double result = -0.0022353984792142715*x*x + 1.2784362463339942*x + 1.236184555478578; // CURVE:real,09:19,03/29
        return result;
    }

     public static double visToRealYaw(double ang) {

        double x = ang;
        double result = 0.0017841955221501891*x*x + 1.1980512961716467*x + 0.47031697208479567; // CURVE:realyaw,09:50,03/29
        return result;
    }

    public List<PhotonTrackedTarget> getTargets() {

        return notes;
    }

    public double getDistanceToTarget(){
        return Util.distance(RobotContainer.driveTrain.getPose(), RobotContainer.targetLocation);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("DistanceTo", getDistanceToTarget());
        notes = noteCamera.getLatestResult().targets;
        poseEstimator.setReferencePose(RobotContainer.driveTrain.getPose()); // sets reference pose to (0,0,
                                                                             // Rotation2d.fromDegrees(0))
        var estimated = poseEstimator.update();
        var result = photonCamera.getLatestResult();

        if (estimated.isPresent() && result.targets.size() >= 2) {

            // var newPose = estimated.get();
            // SmartDashboard.putNumber("Tags Visible", newPose.targetsUsed.size());

            // // RobotContainer.driveTrain.setPose(newPose.estimatedPose.toPose2d());

            // RobotContainer.driveTrain.addVisionMeasurment(
            // newPose.estimatedPose.toPose2d(),
            // ((System.currentTimeMillis()-result.getLatencyMillis())/1000.0));
            // In photonvision, need to have matching photonvision versions, also, need NT
            // connected as well.

        }

        SmartDashboard.putBoolean("Can it see a tag? ", estimated.isPresent());
        PhotonTrackedTarget bestTarget = null;
        //System.out.println("start");
        if(result.hasTargets()){
            bestTarget = result.targets.get(0);
            for(PhotonTrackedTarget possible: result.targets){
                if(possible.getFiducialId() == 9 || possible.getFiducialId() == 10 || possible.getFiducialId() == 9 || possible.getFiducialId() == 9){ continue;}
                //System.out.println("NUMBER"+possible.getFiducialId()+" is "+possible.getYaw());
                if(Math.abs(possible.getYaw()) < Math.abs(bestTarget.getYaw())){
                    bestTarget = possible;
                    //System.out.println("newbest:"+bestTarget.getFiducialId());
                }
            }
        }
        if (result.hasTargets() && Math.abs(bestTarget.getYaw()) < 7) {

            // var bestTarget = result.getBestTarget();
            //System.out.println("used "+bestTarget.getFiducialId());
            var tagPose = layout.getTagPose(bestTarget.getFiducialId());

            double yaw = 180 + visToRealYaw(bestTarget.getYaw()) -
                    RobotContainer.driveTrain.getGyroDegrees();

            TargetCorner bottomCorner = bestTarget.getDetectedCorners().get(0);
            TargetCorner topCorner = bestTarget.getDetectedCorners().get(3);

            double height = bottomCorner.y - topCorner.y;
            double targetsY = topCorner.y + height / 2;
            double pitch = visToRealAngle(bestTarget.getPitch());
            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS,
                    tagPose.get().getZ(),
                    CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(pitch));

            /*Pose2d newPose = PhotonUtils.estimateFieldToRobot(CAMERA_HEIGHT_METERS, tagPose.get().getZ(), CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(bestTarget.getPitch()), Rotation2d.fromDegrees(bestTarget.getYaw()),
                    Rotation2d.fromDegrees(-RobotContainer.driveTrain.getGyroDegrees()), new Pose2d(tagPose.get().getX(), tagPose.get().getY(), new Rotation2d()),
                    new Transform2d(transform3d.getX(), transform3d.getY(), new Rotation2d()));*/

            distanceAverage.put(distance);
            SmartDashboard.putNumber("visDistance", distance);

            double y = Math.sin(Math.toRadians(yaw)) * distanceAverage.get();
            double x = Math.cos(Math.toRadians(yaw)) * distanceAverage.get();

            double robotX = tagPose.get().getX() + -x;
            double robotY = tagPose.get().getY() + y;

            robotX +=(transform3d.getX() *
            Math.cos(RobotContainer.driveTrain.getGyroRadians())) - (transform3d.getY() *
            Math.sin(RobotContainer.driveTrain.getGyroRadians()));
            robotY+= (transform3d.getX() *
            Math.sin(RobotContainer.driveTrain.getGyroRadians())) + (transform3d.getY() *
            Math.cos(RobotContainer.driveTrain.getGyroRadians()));

            RobotContainer.driveTrain.setPose(robotX, robotY, 0);
            //RobotContainer.driveTrain.addVisionMeasurment(new Pose2d(robotX, robotY, new Rotation2d(RobotContainer.driveTrain.getGyroRadians())), System.currentTimeMillis() - result.getLatencyMillis());
            return;
        }

    }
}