package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * MINIMAL TEST to verify FTC Dashboard is working
 * 
 * This sends a simple counter to the dashboard.
 * If you can see the counter incrementing in the Telemetry tab,
 * then your dashboard is working correctly!
 */
@Config
@TeleOp(name = "Dashboard Test", group = "Test")
@Disabled
public class DashboardTest extends LinearOpMode {
    
    // This should appear in the Config tab
    public static double testValue = 123.45;
    
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        
        telemetry.addLine("=== Dashboard Connection Test ===");
        telemetry.addLine("\nConnect to FTC Dashboard:");
        telemetry.addLine("  Phone: 192.168.49.1:8080/dash");
        telemetry.addLine("  Control Hub: 192.168.43.1:8080/dash");
        telemetry.addLine("\nWhat to check:");
        telemetry.addLine("1. Config tab should show 'testValue'");
        telemetry.addLine("2. Telemetry tab should show 'counter'");
        telemetry.addLine("3. Counter should increment each second");
        telemetry.update();
        
        waitForStart();
        
        int counter = 0;
        
        while (opModeIsActive()) {
            // Increment counter
            counter++;
            
            // Send to Driver Station
            telemetry.addData("Counter", counter);
            telemetry.addData("Test Value", testValue);
            telemetry.addLine("\nIf counter is incrementing, OpMode is running!");
            telemetry.addLine("Check Dashboard Telemetry tab for live data");
            telemetry.update();
            
            // Send to Dashboard for graphing
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("counter", counter);
            packet.put("testValue", testValue);
            packet.put("sine_wave", Math.sin(counter * 0.1) * 100);  // Fun to graph!
            dashboard.sendTelemetryPacket(packet);
            
            sleep(100);  // Update 10 times per second
        }
    }
}
