package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * ABSOLUTE MINIMUM dashboard test
 * If THIS flickers, the problem is not your code
 */
@Config
@TeleOp(name = "Super Simple Dash Test", group = "Test")
public class SuperSimpleDashTest extends LinearOpMode {
    
    public static double testValue = 123;
    
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        
        telemetry.addLine("Super Simple Dashboard Test");
        telemetry.addLine("ONE value, SLOW updates");
        telemetry.addLine("If this flickers, problem is:");
        telemetry.addLine("  - Robot Controller WiFi");
        telemetry.addLine("  - Your computer's WiFi");
        telemetry.addLine("  - Dashboard version");
        telemetry.addLine("NOT your code!");
        telemetry.update();
        
        waitForStart();
        
        int counter = 0;
        
        while (opModeIsActive()) {
            counter++;
            
            // Send to Driver Station
            telemetry.addData("Counter", counter);
            telemetry.addData("Test Value", testValue);
            telemetry.update();
            
            // Send to Dashboard - VERY SLOW (once per second)
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("count", counter);
            dashboard.sendTelemetryPacket(packet);
            
            sleep(100);  // 1 second between updates!
        }
    }
}
