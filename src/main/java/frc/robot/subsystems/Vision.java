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
import frc.robot.Constants.Subsystems.VisionConstants;

public class Vision extends SubsystemBase {
	private static Vision s_vision;
	private final PhotonCamera m_camera;
	private final AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFieldLayout
			.loadField(AprilTagFields.k2026RebuiltAndymark);
	private double m_distanceToTags;

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

	private double m_angleToHubTag;

	public void periodic() {
		var results = m_camera.getAllUnreadResults();

		for (var result : results) {
			if (result.hasTargets()) {
				List<PhotonTrackedTarget> targets = result.getTargets();
				double distanceSum = 0;
				double angleSum = 0;
				int numTags = 0;
				for (PhotonTrackedTarget target : targets) {
					if (!VisionConstants.kTrackableTags.contains(target.fiducialId))
						continue;

					numTags++;
					Transform3d botPose = target.getBestCameraToTarget();

					angleSum += target.yaw;

					distanceSum += botPose.getX() +
							Units.inchesToMeters(23.5);
				}

				m_distanceToTags = distanceSum / numTags;
				m_angleToHubTag = angleSum / numTags;

				SmartDashboard.putNumber("Vision/Angle to Tag", m_angleToHubTag);

				SmartDashboard.putNumber(
						"Vision/PoseX", Units.metersToFeet(m_distanceToTags));
			}

		}
	}

	public double getAngleToHubTag() {
		return m_angleToHubTag;
	}

	public double getDistanceToTags() {
		return Units.metersToFeet(m_distanceToTags);
	}

	public double getDistanceToHub() {
		return getDistanceToTags() + (23.5 / 12);
	}
}
