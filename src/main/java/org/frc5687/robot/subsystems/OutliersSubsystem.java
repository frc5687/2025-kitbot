package org.frc5687.robot.subsystems;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.util.BaseInputs;
import org.frc5687.robot.util.BaseOutputs;
import org.frc5687.robot.util.EpilogueLog;

public abstract class OutliersSubsystem<Inputs extends BaseInputs, Outputs extends BaseOutputs>
        extends SubsystemBase implements EpilogueLog {
    protected final SubsystemIO<Inputs, Outputs> _io;
    protected final Inputs _inputs;
    protected final Outputs _outputs;
    protected final RobotContainer _robotContainer;

    private final Object inputLogger;
    private final Object outputLogger;

    @Override
    public String getLogBase() {
        return this.getName();
    }

    public OutliersSubsystem(
            RobotContainer container, SubsystemIO<Inputs, Outputs> io, Inputs inputs, Outputs outputs) {
        _robotContainer = container;
        _io = io;
        _inputs = inputs;
        _outputs = outputs;

        String inputLoggerName = _inputs.getClass().getSimpleName() + "Logger";
        String outputLoggerName = _outputs.getClass().getSimpleName() + "Logger";
        // Hack in fucntinallity such that we can still use Epilogue for logging
        try {
            inputLogger =
                    Class.forName("edu.wpi.first.epilogue.Epilogue")
                            .getField(firstCharToLowerCase(inputLoggerName))
                            .get(null);

            outputLogger =
                    Class.forName("edu.wpi.first.epilogue.Epilogue")
                            .getField(firstCharToLowerCase(outputLoggerName))
                            .get(null);
        } catch (Exception e) {
            throw new RuntimeException("Failed to get loggers", e);
        }
        System.out.println(inputLoggerName);
    }

    public Inputs getInputs() {
        return _inputs;
    }

    protected abstract void processInputs();

    protected abstract void periodic(Inputs inputs, Outputs outputs);

    protected void process() {
        _io.updateInputs(_inputs);
        // reflection to call update() on the logger, We are hacking in functionallity
        // due to Epilgue
        // stuggling to find necessary IO classes
        try {
            inputLogger
                    .getClass()
                    .getMethod("update", EpilogueBackend.class, _inputs.getClass())
                    .invoke(
                            inputLogger, Epilogue.getConfig().backend.getNested(_inputs.getLogPath()), _inputs);
        } catch (Exception e) {
            e.printStackTrace();
        }

        processInputs();
        periodic(_inputs, _outputs);

        try {
            outputLogger
                    .getClass()
                    .getMethod("update", EpilogueBackend.class, _outputs.getClass())
                    .invoke(
                            outputLogger,
                            Epilogue.getConfig().backend.getNested(_outputs.getLogPath()),
                            _outputs);
        } catch (Exception e) {
            e.printStackTrace();
        }

        _io.writeOutputs(_outputs);
    }

    @Override
    public final void periodic() {
        process();
    }

    private String firstCharToLowerCase(String str) {
        return str.substring(0, 1).toLowerCase() + str.substring(1);
    }
}
