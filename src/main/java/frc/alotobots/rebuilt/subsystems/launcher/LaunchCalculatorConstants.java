package frc.alotobots.rebuilt.subsystems.launcher;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.alotobots.util.UnitInterpolatingMap;
import lombok.experimental.UtilityClass;

import static edu.wpi.first.units.Units.*;


/**
 * @see LaunchCalculator
 * */
@UtilityClass
public final class LaunchCalculatorConstants {
    //TODO: Get real data for this
    public static final Distance MINIMUM_SHOOTING_DISTANCE = Meters.of(1);
    public static final Distance MAXIMUM_SHOOTING_DISTANCE = Meters.of(5);
    /**
     * The minimum distance change (in meters) between consecutive lookahead iterations before the
     * solution is considered converged and the loop exits early.
     */
    public static final Distance CONVERGED_DISTANCE_ESTIMATE_THRESHOLD = Millimeters.of(1);
    
    /**
     * How long the subsystem and code takes to respond to a command. Should be empirically tuned.
     * */
    public static final Time LAUNCH_CALCULATOR_SUBSYSTEM_DELAY = Milliseconds.of(30); 
    
    public final class Maps {
        public static final UnitInterpolatingMap<DistanceUnit, AngleUnit> LAUNCHER_DEFLECTOR_ANGLE_MAP =
                new UnitInterpolatingMap<>(Units.Meters, Units.Radians);
        public static final UnitInterpolatingMap<DistanceUnit, AngularVelocityUnit> LAUNCHER_SHOOTER_VELOCITY_MAP =
                new UnitInterpolatingMap<>(Units.Meters, Units.RevolutionsPerSecond);
        public static final UnitInterpolatingMap<DistanceUnit, TimeUnit> FUEL_TIME_OF_FLIGHT_MAP =
                new UnitInterpolatingMap<>(Units.Meters, Units.Seconds);
        
        //TODO: real data
        static {
            // Deflector
            LAUNCHER_DEFLECTOR_ANGLE_MAP.put(Meters.of(0), Degrees.of(68));
            
            // Shooter
            LAUNCHER_SHOOTER_VELOCITY_MAP.put(Meters.of(0), RotationsPerSecond.of(35));
            
            // Time of Flight
            FUEL_TIME_OF_FLIGHT_MAP.put(Meters.of(0), Seconds.of(.5));
        }
    }
}
