package org.frc5687.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.subsystems.OutliersSubsystem;

public class DriveSubsystem extends OutliersSubsystem<DriveInputs, DriveOutputs> {
    private final RobotContainer _container;
    private final DriveIO _driveIO;
    @NotLogged private final SwerveDriveKinematics _kinematics;
    @NotLogged private final SwerveDriveOdometry _odometry;

    private final Translation2d[] _moduleLocations;
    private final SwerveSetpointGenerator _setpointGenerator;
    private SwerveSetpoint _currentSetpoint;

    private final RobotConfig _robotConfig;

    public DriveSubsystem(RobotContainer container, DriveIO io, Translation2d[] moduleLocations) {
        super(container, io, new DriveInputs(), new DriveOutputs());
        _container = container;
        _driveIO = io;
        _moduleLocations = moduleLocations;

        _kinematics = new SwerveDriveKinematics(moduleLocations);
        _odometry = new SwerveDriveOdometry(_kinematics, _inputs.yawPosition, _inputs.modulePositions);

        _robotConfig =
                new RobotConfig(
                        Units.Kilograms.of(Constants.DriveTrain.ROBOT_WEIGHT),
                        Units.KilogramSquareMeters.of(8.085),
                        new ModuleConfig(
                                Constants.SwerveModule.WHEEL_RADIUS,
                                Constants.SwerveModule.MAX_LINEAR_SPEED,
                                Constants.SwerveModule.COEFFICIENT_OF_FRICTION,
                                DCMotor.getKrakenX60Foc(1).withReduction(Constants.SwerveModule.GEAR_RATIO_DRIVE),
                                Constants.SwerveModule.DRIVE_CURRENT_LIMIT,
                                1),
                        moduleLocations[0],
                        moduleLocations[1],
                        moduleLocations[2],
                        moduleLocations[3]);

        _setpointGenerator =
                new SwerveSetpointGenerator(
                        _robotConfig,
                        DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec
                                / Constants.SwerveModule.GEAR_RATIO_STEER
                        );

        _currentSetpoint =
                new SwerveSetpoint(
                        new ChassisSpeeds(),
                        _inputs.measuredStates,
                        DriveFeedforwards.zeros(_robotConfig.numModules));

    }

    @Override
    protected void processInputs() {
        _inputs.odometryPose = _odometry.update(_inputs.yawPosition, _inputs.modulePositions);
    }

    @Override
    protected void periodic(DriveInputs inputs, DriveOutputs outputs) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            log("alliance", alliance.get());
        } else {
            log("alliance", "empty");
        }

        updateSetpoint();
        outputs.desiredStates = _currentSetpoint.moduleStates();
    }

    public void setDesiredChassisSpeeds(ChassisSpeeds speeds) {
        // if (_enableCOMLimiter) {
        //     ChassisSpeeds limitedSpeeds = _comLimiter.limitSpeeds(speeds);
        //     _outputs.desiredSpeeds = limitedSpeeds;
        // } else {
        _outputs.desiredSpeeds = speeds;
        // }
    }

    private void updateSetpoint() {
        _currentSetpoint =
                _setpointGenerator.generateSetpoint(
                        _currentSetpoint, _outputs.desiredSpeeds, Constants.UPDATE_PERIOD);
    }

    public Rotation2d getHeading() {
        return _inputs.yawPosition;
    }

    public void zeroIMU() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            _driveIO.setYaw(Rotation2d.fromDegrees(180));
        } else {
            _driveIO.setYaw(Rotation2d.fromDegrees(0));
        }
    }

    public double getAngularVelocityYaw() {
        return _inputs.yawVelocityRadPerSec;
    }

    public ChassisSpeeds getMeasuredChassisSpeeds() {
        return _kinematics.toChassisSpeeds(_inputs.measuredStates);
    }

    public SwerveDriveKinematics getKinematics() {
        return _kinematics;
    }

    public Translation2d[] getModuleLocations() {
        return _moduleLocations;
    }

    public SwerveModulePosition[] getModulePositions() {
        return _inputs.modulePositions;
    }

    public void runCharacterization(double output) {
        _driveIO.runCharacterization(output);
    }

}