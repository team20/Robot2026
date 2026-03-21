package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PoseUtils;

public class Vision extends SubsystemBase {
	private static Vision s_vision;
	private final PhotonCamera m_camera;
	private final StructPublisher<Pose2d> m_estimatedPoseTopic = NetworkTableInstance.getDefault()
			.getStructTopic("Estimated Camera Pose", Pose2d.struct)
			.publish(); // Initialize network table topic for estimated vision pose

	public Vision() {
		if (s_vision == null) {
			s_vision = this;
		} else {
			throw new Error("Vision already instantiated");
		}

		m_camera = new PhotonCamera("TurretCamera");
	}

	public static Vision getVision() {
		return s_vision;
	}

	public static PhotonCamera getCamera() {
		return s_vision.m_camera;
	}

	public void periodic() {
		List<PhotonPipelineResult> result = Vision.getCamera().getAllUnreadResults();
		if (result.isEmpty()) {
			return;
		}
		PhotonPipelineResult latest = result.get(result.size() - 1);
		Map<Integer, PhotonTrackedTarget> tags = new HashMap<>();
		for (PhotonTrackedTarget target : latest.getTargets()) {
			tags.put(target.getFiducialId(), target);
		}

		if (tags.containsKey(11)) {
			SmartDashboard.putNumber("Tag 11 angle", tags.get(11).getYaw());
		}

		if (tags.containsKey(2)) {
			SmartDashboard.putNumber("Tag 2 angle", tags.get(2).getYaw());
		}
		/*
		 * latest.getTargets().stream().filter(tag -> tag.getFiducialId() ==
		 * 11).findFirst().ifPresent(tag -> {
		 * SmartDashboard.putNumber("Vision/Tag Angle", tag.getYaw());
		 * });
		 */
		PoseUtils.estimatePoseWithStdDev(latest).ifPresent(pose -> {
			m_estimatedPoseTopic.accept(pose.pose());
			SmartDashboard.putNumber("Estimate Pose Deviation/x", pose.xStdDev());
			SmartDashboard.putNumber("Estimate Pose Deviation/y", pose.yStdDev());
			SmartDashboard.putNumber(
					"Estimate Pose Deviation/theta",
					pose.thetaStdDev());
		});
	}
}
