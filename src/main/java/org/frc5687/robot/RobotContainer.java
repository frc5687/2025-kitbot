package org.frc5687.robot;

import org.frc5687.robot.util.EpilogueLog;
import org.frc5687.robot.util.Helpers;
import org.frc5687.robot.commands.drive.TeleopDriveCommand;
import org.frc5687.robot.subsystems.drive.CTREDriveIO;
import org.frc5687.robot.subsystems.drive.DriveIO;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.drive.SimDriveIO;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer implements EpilogueLog {

    private final Robot _robot;
    private final OperatorInterface _oi;
    private final DriveSubsystem _drive;

    public RobotContainer(Robot robot) {
        _robot = robot;
        _oi = new OperatorInterface();

        DriveIO driveIO =
                RobotBase.isSimulation()
                        ? new SimDriveIO(RobotMap.CAN.PIGEON.PIGEON)
                        : new CTREDriveIO(RobotMap.CAN.PIGEON.PIGEON, Constants.SwerveModule.CAN_BUS);

        _drive = new DriveSubsystem(this, driveIO, Constants.DriveTrain.MODULE_LOCATIONS);

        _oi.configureCommandMapping(this);

        configureDefaultCommands();
    }
    private void configureDefaultCommands() {
        _drive.setDefaultCommand(
                new TeleopDriveCommand(
                        _drive,
                        () -> -modifyAxis(_oi.getDriverController().getLeftY()) * Constants.DriveTrain.MAX_MPS,
                        () -> -modifyAxis(_oi.getDriverController().getLeftX()) * Constants.DriveTrain.MAX_MPS,
                        () -> -modifyAxis(_oi.getDriverController().getRightX()) * Constants.DriveTrain.MAX_MPS,
                        () -> true // Always field relative
                        ));

       }

    public Command getAutonomousCommand() {
        return null;
    }

    public void periodic() {
        
    }

    public static double modifyAxis(double value) {
        value = Helpers.applyDeadband(value, 0.1);
        value = Math.copySign(value * value, value);

        return value;
    }

    public DriveSubsystem getDrive() {
        return _drive;
    }

    @Override
    public String getLogBase() {
        return "RobotContainer";
    }
}
