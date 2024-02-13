// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

import java.util.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
    private static PhoenixOdometryThread instance = null;
    private final Lock signalsLock =
            new ReentrantLock(); // Prevents conflicts when registering signals
    private final List<StatusSignal<Double>> signals = new ArrayList<>();
    private final List<StatusSignal<Double>> signalSlopes = new ArrayList<>();
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();
    private boolean isCANFD = false;

    private PhoenixOdometryThread() {
        setName("PhoenixOdometryThread");
        setDaemon(true);
    }

    public static PhoenixOdometryThread getInstance() {
        if (instance == null) {
            instance = new PhoenixOdometryThread();
        }
        return instance;
    }

    @Override
    public void start() {
        if (!timestampQueues.isEmpty()) {
            super.start();
        }
    }

    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayDeque<>(100);
        SwerveDrive.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            SwerveDrive.odometryLock.unlock();
        }
        return queue;
    }

    public Queue<Double> registerSignal(
            ParentDevice device, StatusSignal<Double> signal, StatusSignal<Double> signalSlope) {
        Queue<Double> queue = new ArrayDeque<>(100);
        signalsLock.lock();
        SwerveDrive.odometryLock.lock();
        try {
            isCANFD = CANBus.isNetworkFD(device.getNetwork());

            signals.add(signal);
            signalSlopes.add(signalSlope);

            queues.add(queue);
        } finally {
            signalsLock.unlock();
            SwerveDrive.odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        List<StatusSignal<Double>> all = new ArrayList<>();
        all.addAll(signals);
        all.addAll(signalSlopes);
        BaseStatusSignal[] allArr = all.toArray(new BaseStatusSignal[0]);

        while (true) {
            // Wait for updates from all signals
            signalsLock.lock();
            try {
                if (isCANFD) {
                    BaseStatusSignal.waitForAll(2.0 / SwerveConstants.ODOMETRY_FREQUENCY, allArr);
                } else {
                    // "waitForAll" does not support blocking on multiple
                    // signals with a bus that is not CAN FD, regardless
                    // of Pro licensing. No reasoning for this behavior
                    // is provided by the documentation.
                    sleep((long) (1000.0 / SwerveConstants.ODOMETRY_FREQUENCY));
                    if (!signals.isEmpty()) {
                        signals.forEach(StatusSignal::refresh);
                        signalSlopes.forEach(StatusSignal::refresh);
                    }
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            // Save new data to queues
            SwerveDrive.odometryLock.lock();
            try {
                double timestamp = Logger.getRealTimestamp() / 1e6;
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : signals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (!signals.isEmpty()) {
                    timestamp -= totalLatency / signals.size();
                }
                for (int i = 0; i < signals.size(); i++) {
                    double value =
                            BaseStatusSignal.getLatencyCompensatedValue(
                                    signals.get(i), signalSlopes.get(i));
                    queues.get(i).offer(value);
                }
                for (Queue<Double> timestampQueue : timestampQueues) {
                    timestampQueue.offer(timestamp);
                }
            } finally {
                SwerveDrive.odometryLock.unlock();
            }
        }
    }
}
