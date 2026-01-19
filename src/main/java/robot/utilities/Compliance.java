package robot.utilities;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.reflect.Field;

/**
 * A class which keeps track of places in the code where values have not yet
 * been decided and need to be fixed at a later time.
 */
public class Compliance {
	/**
	 * An annotation which can be put on any value in order to tag it as incomplete.
	 */
	@Retention(RetentionPolicy.RUNTIME)
	public static @interface FixMe {
		String reason() default "TBD";
	}

	/**
	 * An error to throw when a value tagged as incomplete is requested.
	 */
	public static class ComplianceError extends Error {
		public ComplianceError(FixMe fixme) {
			super(String.format("Code tagged as non-compliant: %s", fixme.reason()));
		}
	}

	/**
	 * Gets the value of a static field in a object which may or may not be tagged
	 * as incomplete.
	 * 
	 * @param <T> type of the field value
	 * @param object class which stores the field
	 * @param key name of the field
	 * @return value of the field
	 */
	@SuppressWarnings("unchecked")
	public static <T> T ensure(Class<?> object, String key) {
		Field field = null;
		try {
			field = object.getField(key);
		} catch (NoSuchFieldException | SecurityException e) { // Check to see if the field exists, as it may have been
																// mistyped.
			throw new Error(String.format("Field \"%s\" was not found in class \"%s\"", key, object));
		}
		FixMe annotation = field.getAnnotation(FixMe.class);
		if (annotation != null) { // Check to see if the field is tagged as incomplete.
			throw new ComplianceError(annotation);
		} else {
			try {
				return (T) field.get(null); // Get the value of the field and cast to the desired type.
			} catch (IllegalArgumentException | IllegalAccessException e) { // Check to see if there were issues while
																			// accessing the value of the field. This
																			// shouldn't happen in normal circumstances.
				throw new Error(String.format("Field \"%s\" could not be accessed in class \"%s\"", key, object));
			}
		}
	}
}