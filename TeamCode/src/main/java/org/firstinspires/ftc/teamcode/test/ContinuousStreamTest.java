package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * CONTINUOUS STREAM TEST
 * 
 * This sends data EVERY loop (50 times per second) to keep WebSocket alive.
 * If this doesn't flicker, then your original code wasn't updating frequently enough.
 */
@Config
@TeleOp(name = "Continuous Stream Test", group = "Test")
public class ContinuousStreamTest extends LinearOpMode {
    
    public static int sleepMs = 20;  // Adjustable in dashboard
    
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        
        telemetry.addLine("=== Continuous Stream Test ===");
        telemetry.addLine("\nThis sends data EVERY loop (50/second)");
        telemetry.addLine("to prevent WebSocket timeout.");
        telemetry.addLine("\nIf this DOESN'T flicker:");
        telemetry.addLine("  → Your PID code doesn't update enough");
        telemetry.addLine("  → Solution: Send data every loop");
        telemetry.addLine("\nIf this STILL flickers:");
        telemetry.addLine("  → Update dashboard to 0.4.16");
        telemetry.addLine("  → Check WiFi power settings");
        telemetry.update();
        
        waitForStart();
        
        long counter = 0;
        long startTime = System.currentTimeMillis();
        
        while (opModeIsActive()) {
            long now = System.currentTimeMillis();
            long elapsed = now - startTime;
            
            // Send data EVERY SINGLE LOOP - no throttling!
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("counter", counter);
            packet.put("elapsed_seconds", elapsed / 1000.0);
            packet.put("timestamp", now);
            packet.put("sine_wave", Math.sin(elapsed / 1000.0) * 100);
            dashboard.sendTelemetryPacket(packet);
            
            // Also update Driver Station
            telemetry.addData("Counter", counter);
            telemetry.addData("Elapsed", "%.1f seconds", elapsed / 1000.0);
            telemetry.addData("Updates/sec", counter * 1000.0 / elapsed);
            telemetry.addLine("\nWatch dashboard - should NOT flicker!");
            telemetry.update();
            
            counter++;
            sleep(sleepMs);
        }
    }
}
