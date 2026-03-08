package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
	private final PhotonCamera m_camera;
	private final AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFieldLayout
			.loadField(AprilTagFields.k2026RebuiltAndymark);
	private double m_distanceToTags;
	// private final Transform3d botToCam = new Transform3d(new Transform2d(0, 0,
	// Rotation2d.fromDegrees(0)));
	// private final PhotonPoseEstimator m_poseEstimator = new
	// PhotonPoseEstimator(m_aprilTagFieldLayout, botToCam);

	public Vision() {
		m_camera = new PhotonCamera("TestCamLBord");
	}

	public void periodic() {
		var results = m_camera.getAllUnreadResults();
		for (var result : results) {
			if (result.hasTargets()) {
				List<PhotonTrackedTarget> targets = result.getTargets();
				double sum = 0;
				for (PhotonTrackedTarget target : targets) {
					Transform3d botPose = target.getBestCameraToTarget();
					sum += botPose.getX() +
							Units.inchesToMeters(23.5);
				}
				m_distanceToTags = sum / targets.size();

				SmartDashboard.putNumber(
						"Vision/PoseX", Units.metersToFeet(m_distanceToTags));
			}
			/*
			 * // var multiTagResult = result.getMultiTagResult();
			 * SmartDashboard.putBoolean("Vision/Has Results", result.hasTargets());
			 * // if (multiTagResult.isPresent()) {
			 * if (result.hasTargets()) {
			 * Optional<EstimatedRobotPose> visionEst =
			 * m_poseEstimator.estimateCoprocMultiTagPose(result);
			 * if (visionEst.isEmpty()) {
			 * visionEst = m_poseEstimator.estimateLowestAmbiguityPose(result);
			 * }
			 * Pose3d tagPose = m_aprilTagFieldLayout.getTagPose(4).get();
			 * Pose3d hubPose = new Pose3d(new Pose2d(tagPose.getX() - 23.5, tagPose.getY(),
			 * new Rotation2d()));
			 * Pose3d estimate = visionEst.get().estimatedPose;
			 * Translation3d poseDiff =
			 * hubPose.getTranslation().minus(estimate.getTranslation());
			 * // multiTagResult.get().estimatedPose.best.getTranslation());
			 * SmartDashboard.putNumber("Vision/PoseX", estimate.getX());
			 * SmartDashboard.putNumber("Vision/PoseY", estimate.getY());
			 * SmartDashboard.putNumber("Vision/PoseZ", estimate.getZ());
			 * }
			 */

			// SmartDashboard.putNumber("Vision/PoseY", botPose.getY());
			// SmartDashboard.putNumber("Vision/PoseZ", botPose.getZ());

			// int id = target.getFiducialId();
			// var tagPose = m_aprilTagFieldLayout.getTagPose(id);
			// if (tagPose.isPresent()) {
			// Pose3d botPose = PhotonUtils
			// .estimateFieldToRobotAprilTag(
			// target.getBestCameraToTarget(), tagPose.get(),
			// new Transform3d(new Transform2d(0, 0, Rotation2d.fromDegrees(0))));
			// SmartDashboard.putNumber(
			// "Vision/PoseX", botPose.getX() +
			// Units.inchesToMeters(23.5));
			// SmartDashboard.putNumber("Vision/PoseY", botPose.getY());
			// SmartDashboard.putNumber("Vision/PoseZ", botPose.getZ());
			// }
		}
	}

	public double getDistanceToTags() {
		return Units.metersToFeet(m_distanceToTags);
	}

	public double getDistanceToHub() {
		return getDistanceToTags() + (23.5 / 12);
	}
}
