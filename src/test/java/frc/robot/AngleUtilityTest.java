package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class AngleUtilityTest {
	@ParameterizedTest(name = "minDifference({0}, {1}) = {2}")
	@CsvSource({
			"0, 0, 0",
			"10, 30, 20",
			"30, 10, 20", // order doesn't matter
			"350, 10, 20", // wraps around 0/360
			"10, 350, 20",
			"0, 180, 180", // opposite sides
			"-90, 90, 180",
	})
	void minDifference(double a, double b, double expected) {
		assertEquals(expected, AngleUtility.minDifference(a, b), 1e-9);
	}
}
