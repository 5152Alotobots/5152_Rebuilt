package frc.alotobots.rebuilt.subsystems.intake.extendo.constants;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import lombok.experimental.UtilityClass;

import static edu.wpi.first.units.Units.*;

@UtilityClass
public class IntakeExtendoConstants {
    
    public static final class Thresholds {
        /** Acceptable PID error that will classify as "at position" */
        public static final Distance AT_TARGET_EXTENSION_POSITION_THRESHOLD = Meters.of(.02);

        /** How long the extendo must be "at position" to classify as "at position" */
        public static final Time AT_TARGET_EXTENSION_TIME_THRESHOLD = Seconds.of(.2);
    }

    /** Physical limits and safety thresholds */
    public static final class Limits {
        /** Maximum allowed extension */
        public static final Distance MAX_EXTENSION = Meters.of(1.80);

        /** Minimum allowed extension */
        public static final Distance MIN_EXTENSION = Meters.of(0.23);
        
        /** Maximum open loop percent output */
        public static final double MAX_OPEN_LOOP_PERCENTAGE = 0.5;

        /** Max speed (magnitude) */
        public static final LinearVelocity MAX_OPERATOR_VELOCITY = MetersPerSecond.of(0.5);

        /** Enable Limits */
        public static final boolean LIMITS_ENABLED = true;

        /** Maximum velocity near the top or bottom limit */
        public static final LinearVelocity MAX_VELOCITY_NEAR_LIMIT = MetersPerSecond.of(0.3);

        /** Distance from the top or bottom limit where the velocity limit applies */
        public static final Distance DISTANCE_FROM_LIMIT = Meters.of(0.2);
    }

    /** Position setpoints for different extendo states */
    public static final class Setpoints {
        /** Extension when extendo is fully retracted/stowed */
        public static final Distance STOWED = Meters.of(0.1);

        /** Extension when the extendo is fully deployed*/
        public static final Distance DEPLOYED = Meters.of(1.1);
    }
}
