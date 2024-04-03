package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionSubsystem {
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    PhotonCamera leftCam = new PhotonCamera("leftCam");
    Transform3d robotToCam = new Transform3d(new Translation3d(0.250825, 0.21155, -0.2921), new Rotation3d(0.0, 28.125, 30));
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCam, robotToCam);
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return photonPoseEstimator.update();
    }
}
