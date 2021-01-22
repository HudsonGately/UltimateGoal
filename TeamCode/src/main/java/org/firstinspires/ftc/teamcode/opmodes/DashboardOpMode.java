package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;

public abstract class DashboardOpMode extends CommandOpMode {
    protected FtcDashboard dashboard = FtcDashboard.getInstance();
    protected TelemetryPacket packet = new TelemetryPacket();

    public void updateTelemetry() {
        super.updateTelemetry(telemetry);
        dashboard.sendTelemetryPacket(packet);
    }
}
