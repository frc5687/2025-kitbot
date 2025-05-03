package org.frc5687.robot.commands.elevator;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;
import org.frc5687.robot.util.ReefAlignmentHelpers;

public class GoToAlgaeHeight extends OutliersCommand {
    private final ElevatorSubsystem _elevator;
    private final DriveSubsystem _drive;
    private final ElevatorState _algaeHeight;

    public GoToAlgaeHeight(ElevatorSubsystem elevator, DriveSubsystem drive) {
        _elevator = elevator;
        _drive = drive;
        _algaeHeight = ElevatorState.HIGH_ALGAE_GRAB;
        addRequirements(_elevator);
    }

    @Override
    public void initialize() {
        _elevator.setDesiredHeight(_algaeHeight);
    }

    @Override
    public void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        return _elevator.isAtDesiredPosition();
        // return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
