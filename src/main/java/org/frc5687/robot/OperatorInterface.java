package org.frc5687.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.frc5687.robot.commands.elevator.GoToAlgaeHeight;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem

;

public class OperatorInterface {
    private final CommandPS5Controller _driverController;
    private DriveSubsystem _drive;
    private ElevatorSubsystem _elevator;

    public OperatorInterface(DriveSubsystem drive, ElevatorSubsystem elevator) {
        _driverController = new CommandPS5Controller(0);
        _drive = drive;
        _elevator = elevator;
    }

    public void configureCommandMapping(RobotContainer container) {
        // _driverController.povDown().onTrue(Commands.runOnce(drive::zeroGyroscope));
         GoToAlgaeHeight goToAlgaeHeight = new GoToAlgaeHeight(_elevator,_drive);
        _driverController.triangle().onTrue(goToAlgaeHeight);
        // _driverController.b().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // _driverController.y().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // _driverController.x().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    }

    public CommandPS5Controller getDriverController() {
        return _driverController;
    }
}
