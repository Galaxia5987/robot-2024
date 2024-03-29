// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.webconstants;

import frc.robot.lib.GeneralRobotLoop;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber {

    private static final String tableKey = "TunableNumbers";
    public static boolean IN_TUNING_MODE = true;
    private final String key;
    private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();
    private boolean hasDefault = false;
    private double defaultValue;
    private LoggedDashboardNumber dashboardNumber;

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public LoggedTunableNumber(String dashboardKey) {
        this.key = tableKey + "/" + dashboardKey;
    }

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public LoggedTunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(double defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (IN_TUNING_MODE) {
                dashboardNumber = new LoggedDashboardNumber(key, defaultValue);
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public double get() {
        if (!hasDefault) {
            return 0.0;
        } else {
            if (IN_TUNING_MODE) {
                return dashboardNumber.get();
            } else {
                return defaultValue;
            }
        }
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false
     *     otherwise.
     */
    public boolean hasChanged() {
        double currentValue = get();
        Double lastValue = lastHasChangedValues.get(0);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(0, currentValue);
            return true;
        }

        return false;
    }

    public void ifChanged(int hash, DoubleConsumer action) {
        GeneralRobotLoop.register(
                () -> {
                    if (hasChanged()) {
                        action.accept(get());
                    }
                });
    }

    public static void ifChanged(
            int hash, Consumer<double[]> action, LoggedTunableNumber... numbers) {
        GeneralRobotLoop.register(
                () -> {
                    if (Arrays.stream(numbers).allMatch((number) -> number.hasChanged())) {
                        action.accept(
                                Arrays.stream(numbers)
                                        .mapToDouble(LoggedTunableNumber::get)
                                        .toArray());
                    }
                });
    }
}
