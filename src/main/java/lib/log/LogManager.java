package lib.log;

import java.util.ArrayList;
import java.util.List;

public class LogManager {

    private static LogManager instance;

    private final List<Loggable> subsystems = new ArrayList<>();

    public static LogManager getInstance() {
        if(instance == null)
            instance = new LogManager();

        return instance;
    }

    private LogManager() {}

    public void addSubsystem(Loggable subsystem){
        subsystems.add(subsystem);
    }

    public void displayLogs() {
        for(Loggable subsystem : subsystems){
            subsystem.log();
        }
    }

    public void disableLogs() {
        subsystems.clear();
    }
}
