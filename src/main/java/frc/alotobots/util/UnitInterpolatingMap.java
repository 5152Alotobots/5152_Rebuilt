package frc.alotobots.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class UnitInterpolatingMap<K extends Unit, V extends Unit> {
    private final InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    private final K keyUnit;
    private final V valueUnit;

    public UnitInterpolatingMap(K keyUnit, V valueUnit) {
        this.keyUnit = keyUnit;
        this.valueUnit = valueUnit;
    }

    public void put(Measure<K> key, Measure<V> value) {
        map.put(key.in(keyUnit), value.in(valueUnit));
    }

    @SuppressWarnings("unchecked")
    public Measure<V> get(Measure<K> key) {
        return (Measure<V>) valueUnit.of(map.get(key.in(keyUnit)));
    }
}