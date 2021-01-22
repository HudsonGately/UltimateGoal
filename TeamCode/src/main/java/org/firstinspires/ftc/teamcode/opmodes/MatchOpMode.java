package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.RunCommand;

public abstract class MatchOpMode extends DashboardOpMode {
    @Override
    public void initialize() {
        robotInit();
        configureButtons();
        while(!isStarted() && !isStopRequested()) {
            disabledPeriodic();
            robotPeriodic();
            updateTelemetry();
        }
    }

    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        schedule(new RunCommand(() -> {
            updateTelemetry();
            robotPeriodic();
        }));
        matchStart();
        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
            matchLoop();
        }
        reset();
    }

    public abstract void robotInit();
    public void configureButtons() {};
    public void disabledPeriodic() {};
    public abstract void matchStart();
    public void matchLoop() {};
    public void robotPeriodic() {};
}
