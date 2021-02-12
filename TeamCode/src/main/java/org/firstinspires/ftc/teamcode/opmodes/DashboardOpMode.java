package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;

import java.util.logging.Level;
import java.util.logging.Logger;

public abstract class DashboardOpMode extends CommandOpMode {
    protected FtcDashboard dashboard = FtcDashboard.getInstance();

    protected TelemetryPacket packet = new TelemetryPacket();

    public void updateTelemetry() {
        super.updateTelemetry(telemetry);
        Logger.getLogger("Dashboard").log(Level.WARNING, "Packet Data", packet);
    }
}
