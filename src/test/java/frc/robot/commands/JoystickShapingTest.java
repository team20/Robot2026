package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands.JoystickDrive;

/**
 * Checks how the drive stick becomes drive power ({@link JoystickDrive#shapeTranslation}): the stick is
 * treated as one arrow, so the deadband and curve change only how far it's pushed, never which way.
 */
class JoystickShapingTest {
	private static final double kTolerance = 1e-9;

	private static double angleDegrees(Translation2d t) {
		return Math.toDegrees(Math.atan2(t.getY(), t.getX()));
	}

	@Test
	void centeredStickGivesNoPower() {
		Translation2d out = JoystickDrive.shapeTranslation(0, 0);
		assertEquals(0, out.getNorm(), kTolerance);
	}

	@Test
	void smallPushInAnyDirectionIsIgnored() {
		// 0.042 long, inside the 0.05 round deadband
		assertEquals(0, JoystickDrive.shapeTranslation(0.03, 0.03).getNorm(), kTolerance);
		assertEquals(0, JoystickDrive.shapeTranslation(-0.049, 0).getNorm(), kTolerance);
	}

	@Test
	void powerStartsFromZeroJustOutsideTheDeadband() {
		assertTrue(JoystickDrive.shapeTranslation(DriveConstants.kDeadzone + 1e-4, 0).getNorm() < 1e-3,
				"no jump in power at the edge of the deadband");
	}

	@Test
	void fullPushIsFullPowerStraight() {
		Translation2d forward = JoystickDrive.shapeTranslation(1, 0);
		assertEquals(1, forward.getX(), kTolerance);
		assertEquals(0, forward.getY(), kTolerance);
		Translation2d backward = JoystickDrive.shapeTranslation(-1, 0);
		assertEquals(-1, backward.getX(), kTolerance);
		Translation2d left = JoystickDrive.shapeTranslation(0, 1);
		assertEquals(1, left.getY(), kTolerance);
	}

	@Test
	void fullPushIsFullPowerOnDiagonals() {
		// A round stick gate reads about (0.71, 0.71) at a full diagonal push. The old per-axis curve
		// turned that into 0.71 total power.
		double d = Math.sqrt(0.5);
		Translation2d out = JoystickDrive.shapeTranslation(d, d);
		assertEquals(1, out.getNorm(), 1e-6);
		assertEquals(45, angleDegrees(out), 1e-6);
	}

	@Test
	void stickCornersAreNoFasterThanAFullPush() {
		// Some gamepads report (1, 1) in the corners, which is 1.41 long
		Translation2d out = JoystickDrive.shapeTranslation(1, 1);
		assertEquals(1, out.getNorm(), 1e-6);
		assertEquals(45, angleDegrees(out), 1e-6);
		assertEquals(1, JoystickDrive.shapeTranslation(-1, -1).getNorm(), 1e-6);
	}

	@Test
	void robotGoesTheWayTheStickPoints() {
		// The old per-axis curve bent this 18.4 degree push to about 15 degrees
		Translation2d out = JoystickDrive.shapeTranslation(0.9, 0.3);
		assertEquals(Math.toDegrees(Math.atan2(0.3, 0.9)), angleDegrees(out), 1e-6);
		Translation2d backLeft = JoystickDrive.shapeTranslation(-0.4, 0.7);
		assertEquals(Math.toDegrees(Math.atan2(0.7, -0.4)), angleDegrees(backLeft), 1e-6);
	}

	@Test
	void slightlyOffAxisPushesDoNotSnapToStraight() {
		// The old per-axis deadband zeroed the 0.04 sideways part, so the robot drove straight forward
		Translation2d out = JoystickDrive.shapeTranslation(0.6, 0.04);
		assertTrue(out.getY() > 0, "keeps the small sideways part");
		assertEquals(Math.toDegrees(Math.atan2(0.04, 0.6)), angleDegrees(out), 1e-6);
	}

	@Test
	void halfPushFollowsTheResponseCurve() {
		// Deadband rescale: (0.5 - 0.05) / 0.95 = 0.4737, then raised to the exponent
		double expected = Math.pow((0.5 - DriveConstants.kDeadzone) / (1 - DriveConstants.kDeadzone),
				DriveConstants.kTranslationInputExponent);
		assertEquals(expected, JoystickDrive.shapeTranslation(0.5, 0).getX(), kTolerance);
		assertEquals(-expected, JoystickDrive.shapeTranslation(0, -0.5).getY(), kTolerance);
	}
}
