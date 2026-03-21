package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PoseUtils;

public class Vision extends SubsystemBase {
	private static Vision s_vision;
	private final PhotonCamera m_camera;
	private double m_distance;
	private double m_angle;
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
		PoseUtils.estimateCamPoseStdDev(latest).ifPresent(pose -> {
			Translation2d difference = PoseUtils.getHub().minus(pose.pose()).getTranslation();
			m_angle = -difference.getAngle().getDegrees();
			m_distance = Units.metersToFeet(difference.getNorm());
			m_estimatedPoseTopic.accept(pose.pose());
			SmartDashboard.putNumber("Angle To Hub", m_angle);
			SmartDashboard.putNumber("Distance To Hub", m_distance);
			SmartDashboard.putNumber("Estimate Pose Deviation/x", pose.xStdDev());
			SmartDashboard.putNumber("Estimate Pose Deviation/y", pose.yStdDev());
			SmartDashboard.putNumber(
					"Estimate Pose Deviation/theta",
					pose.thetaStdDev());
		});
	}

	public double getAngleToHubTag() {
		return m_angle;
	}

	public double getDistanceToHub() {
		return m_distance;
	}
}