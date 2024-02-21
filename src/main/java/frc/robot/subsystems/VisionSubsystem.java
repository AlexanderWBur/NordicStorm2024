package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public boolean canSeeTarget = false;

    public static PhotonPoseEstimator poseEstimator;

    public static RollingAverage distanceAverage = new RollingAverage(5);
    public PhotonTrackedTarget bestTarget = null;
    double camHeight = Units.inchesToMeters(27); // 

    public VisionSubsystem() {

        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        Transform3d transform3d = new Transform3d(new Translation3d(0, -0.0508, -0.368), new Rotation3d(0, 0, 0));

        photonCamera = new PhotonCamera("Arducam_OV9281_USB_Camera"); // Need to find out what to name this
        poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera,
                transform3d);

    }

    @Override
    public void periodic() {
        poseEstimator.setReferencePose(RobotContainer.driveTrain.getPose()); // sets reference pose to (0,0, Rotation2d.fromDegrees(0))
        var estimated = poseEstimator.update();

        if (estimated.isPresent()) {
            var newPose = estimated.get();
            RobotContainer.driveTrain.setPose(newPose.estimatedPose.toPose2d());
            
            // In photonvision, need to have matching photonvision versions, also, need NT connected as well.
        
        }   

        SmartDashboard.putBoolean("Can it see a tag? ",estimated.isPresent());

        
        


    }
}