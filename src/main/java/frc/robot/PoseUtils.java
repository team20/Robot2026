package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.IntToDoubleFunction;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Subsystems.VisionConstants;

public class PoseUtils {
	public record PoseResult(Pose2d pose, double xStdDev, double yStdDev, double thetaStdDev) {
	}

	public record AimResult(double setpoint, double feedforward) {
	}

	private static AprilTagFieldLayout s_layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	/**
	 * Estimate the pose of the robot, including the standard deviations for each
	 * component of the pose. This hopefully may be more reliable than other methods
	 * if we want to implement a cutoff which ignores low quality data with high
	 * standard deviations.
	 * 
	 * @param result a {@code PhotonPiplineResult} which has the latest target data
	 * @return a {@code PoseResult} which contains the {@code Pose2d} of the robot
	 *         and the standard deviations
	 */
	public static Optional<PoseResult> estimateCamPoseStdDev(List<PhotonTrackedTarget> targets) {
		int maxResults = targets.size();
		List<Double> data = new ArrayList<>(maxResults * 4);
		for (PhotonTrackedTarget target : targets) {
			s_layout.getTagPose(target.getFiducialId()).ifPresent(pose -> {
				Pose2d camera = pose.transformBy(target.getBestCameraToTarget().inverse()).toPose2d();
				data.add(camera.getX());
				data.add(camera.getY());
				data.add(camera.getRotation().getSin());
				data.add(camera.getRotation().getCos());
			});
		}
		int count = data.size() / 4;
		if (count == 0) {
			return Optional.empty();
		}
		double x = findCenterOfData(index -> data.get(index * 4), count);
		double y = findCenterOfData(index -> data.get(index * 4 + 1), count);
		double sin = findCenterOfData(index -> data.get(index * 4 + 2), count);
		double cos = findCenterOfData(index -> data.get(index * 4 + 3), count);
		Pose2d pose = new Pose2d(x, y, Rotation2d.fromRadians(Math.atan2(sin, cos)));
		double xStdDev = findStdDevOfData(index -> data.get(index * 4), count, x);
		double yStdDev = findStdDevOfData(index -> data.get(index * 4 + 1), count, y);
		double sinStdDev = findStdDevOfData(index -> data.get(index * 4 + 2), count, sin);
		double cosStdDev = findStdDevOfData(index -> data.get(index * 4 + 3), count, cos);
		return Optional.of(new PoseResult(pose, xStdDev, yStdDev, Math.hypot(sinStdDev, cosStdDev)));
	}

	/**
	 * Finds the standard deviation of a set of data. This function actually
	 * computes the average absolute deviation, but they are essentially the same
	 * for our purposes and standard deviation doesn't work well when there are many
	 * outliers.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @param center the center of the data
	 * @return the standard deviation of the data
	 */
	public static double findStdDevOfData(IntToDoubleFunction data, int size, double center) {
		double deviation = 0;
		for (int i = 0; i < size; i++) {
			deviation += Math.abs(data.applyAsDouble(i) - center);
		}
		return deviation / size * Math.sqrt(Math.PI / 2);
	}

	/**
	 * Finds the center of a set of data. Internally this function uses a
	 * bisection algorithm to find the best center.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @return the best center of the data
	 */
	public static double findCenterOfData(IntToDoubleFunction data, int size) {
		return findCenterOfData(data, size, 5);
	}

	/**
	 * Finds the center of a set of data. It has a data-independent tunable
	 * parameter called {@code outlierRejectionAbility} which determines whether the
	 * center is closer to the mean or the median. Internally this function uses a
	 * bisection algorithm to find the best center.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @param outlierRejectionAbility a parameter to tune what types of centers are
	 *        favored
	 * @return the best center of the data
	 */
	public static double findCenterOfData(IntToDoubleFunction data, int size, double outlierRejectionAbility) {
		double min = data.applyAsDouble(0), max = min;
		for (int i = 1; i < size; i++) {
			double point = data.applyAsDouble(i);
			max = Math.max(max, point);
			min = Math.min(min, point);
		}
		double k = Math.exp(-outlierRejectionAbility) * (max - min);
		double high = max, low = min, loss;
		do {
			double bisector = (low + high) / 2;
			loss = loss(data, size, bisector, k);
			if (loss < 0) {
				low = bisector;
			} else {
				high = bisector;
			}
		} while (Math.abs(loss) > 0.001);
		return (low + high) / 2;
	}

	/**
	 * Calculates how good of a center the chosen center is for a given set of data.
	 * When k is increased, the function favors the median. When k is decreased, the
	 * function favors the mean.
	 * 
	 * @param data a way to get the data
	 * @param size how many data points you have
	 * @param center a proposed center for the data
	 * @param k a parameter to tune what types of centers are favored
	 * @return how good the center is
	 */
	private static double loss(IntToDoubleFunction data, int size, double center, double k) {
		double total = 0;
		for (int i = 0; i < size; i++) {
			double difference = center - data.applyAsDouble(i);
			total += difference / Math.hypot(difference, k);
		}
		return total;
	}

	/**
	 * Use this method to the the Pose2d of the hub for the alliance you are on.
	 * 
	 * @return the {@code Pose2d} of the current hub
	 */
	public static Pose2d getHub() {
		return switch (DriverStation.getAlliance().orElse(Alliance.Red)) {
			case Blue -> VisionConstants.kBlueHub;
			case Red -> VisionConstants.kRedHub;
		};
	}
}
