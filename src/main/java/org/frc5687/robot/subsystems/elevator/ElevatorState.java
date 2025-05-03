package org.frc5687.robot.subsystems.elevator;

import org.frc5687.robot.Constants;

public enum ElevatorState {
    STOWED(Constants.Elevator.MIN_HEIGHT),
    PROCESSOR_PLACING(Constants.Elevator.MIN_HEIGHT),
    LOW_ALGAE_GRAB(0.324),
    HIGH_ALGAE_GRAB(0.561 - 0.04),
    BARGE_PLACING(0.65);

    private final double _height;

    ElevatorState(double height) {
        _height = height;
    }

    public double getHeight() {
        return _height;
    }
}
