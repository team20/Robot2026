package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

/** Same cases as ClampedP.testCommand(), but run by ./gradlew test instead of on the robot. */
class ClampedPTest {
	private static final double kMinPower = 1;
	private static final double kMaxPower = 50;
	private static final double kMaxErr = 10;
	private static final double kTolerance = 1;

	@ParameterizedTest(name = "error {0} -> power {1}")
	@CsvSource({
			"-25, 50", // far away: clamped to max power
			"-10, 50",
			"-5.5, 25.5", // in the ramp: proportional
			"-1, 1", // at tolerance: min power
			"-0.5, 0", // inside tolerance: off
			"0, 0",
			"0.5, 0",
			"1, -1",
			"5.5, -25.5",
			"10, -50",
			"25, -50",
	})
	void clampedP(double error, double expected) {
		assertEquals(expected, ClampedP.clampedP(error, kMinPower, kMaxPower, kMaxErr, kTolerance), 1e-9);
	}
}
