package org.frc5687.robot.subsystems.algaearm;

import edu.wpi.first.math.util.Units;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.util.TunableDouble;

public class AlgaeArmSubsystem extends OutliersSubsystem<AlgaeInputs, AlgaeOutputs> {


    public AlgaeArmSubsystem(RobotContainer container, AlgaeArmIO io) {
        super(container, io, new AlgaeInputs(), new AlgaeOutputs());
    }

    @Override
    protected void processInputs() {}

    @Override
    protected void periodic(AlgaeInputs inputs, AlgaeOutputs outputs) {
        
    }

    public void setArmAngle(AlgaeState state) {
        setArmAngle(state.getArmAngle());
    }

    public void setArmAngle(double angle) {
        _outputs.desiredAngleRad = angle;
    }

    public double getArmAngleRads() {
        return _inputs.angleRads;
    }

    public void setAlgaeMotorVoltage(double voltage) {
        _outputs.voltageCommand = voltage;
    }

    public void setWheelMotorVoltage(double voltage) {
        _outputs.wheelVoltageCommand = voltage;
    }

    public boolean isAtDesiredAngle() {
        return Math.abs(_outputs.desiredAngleRad - _inputs.angleRads) < 0.04;
    }

    public boolean isAtState(AlgaeState state) {
        double angleDiff = Math.abs(state.getArmAngle() - getArmAngleRads());
        boolean isWithinPositionTolerance = angleDiff < Units.degreesToRadians(5.0);
        return isWithinPositionTolerance;
    }

    public boolean isSafeToEject() {
        return _inputs.angleRads > Constants.AlgaeArm.BOTTOM_EJECT_SAFE_ANGLE
                || _inputs.angleRads < Constants.AlgaeArm.TOP_EJECT_SAFE_ANGLE;
    }
}
