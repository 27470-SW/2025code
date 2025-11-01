package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.PIDControl;

import java.util.List;

@Config
@TeleOp(name = "TestPidShooter", group = "Test")
public class TestPIDshooter extends InitLinearOpMode
{
    // These can be tuned in FTC Dashboard
    public static double kP = 0.03;  // Start with calculated values
    public static double kI = 0.000016;
    public static double kD = 0.0005;

    public static double kS = 0.0;   // Static friction feedforward
    public static double kV = 0.0003; // Velocity feedforward (start small!)
    public static double kA = 0.0;   // Acceleration feedforward

    public static double targetVelocity = -2000; // ticks per second

    private PIDControl shooterMotor;
    private PIDControl shooterMotor2;

    @Override
    public void runOpMode() {
        // Initialize motor
        shooterMotor = new PIDControl(hardwareMap, "shoot");
        shooterMotor2 = new PIDControl(hardwareMap, "shoot2");

        // Set initial coefficients
        shooterMotor.init(kP, kI, kD, kV);
        shooterMotor2.init(kP, kI, kD, kV);

        telemetry.addLine("Connect to FTC Dashboard:");
        telemetry.addLine("192.168.49.1:8080/dash (phone)");
        telemetry.addLine("192.168.43.1:8080/dash (Control Hub)");
        telemetry.addLine("\nPress A to spin up, B to stop");
        telemetry.update();

        waitForStart();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashTelemetry = dashboard.getTelemetry();

        while (opModeIsActive()) {
            // Update coefficients from Dashboard in real-time
            shooterMotor.init(kP, kI, kD, kV);
            shooterMotor2.init(kP, kI,kD,kV);
            double update = shooterMotor.update();
            double update2 = shooterMotor2.update();

            // Control motor
            if (gamepad1.a) {
                shooterMotor.setWheelVelocity(targetVelocity);
            } else if (gamepad1.b) {
                shooterMotor.setWheelVelocity(0);
            }


            // Get current velocity
            double currentVelocity = shooterMotor.getVelocity();
            double error = targetVelocity - currentVelocity;

            // Send data to Dashboard for graphing

            dashTelemetry.addData("targetVelocity", targetVelocity);
            dashTelemetry.addData("currentVelocity", currentVelocity);
            dashTelemetry.addData("error", error);
            dashTelemetry.addData("kP", kP);
            dashTelemetry.addData("kI", kI);
            dashTelemetry.addData("kD", kD);
            dashTelemetry.addData("kV", kV);
            dashTelemetry.addData("Power", update);
            dashTelemetry.addData("filteredVelocity", shooterMotor.getFilteredVelocity());
            dashTelemetry.update();

            // Also show on Driver Station
            telemetry.addData("Target", targetVelocity);
            telemetry.addData("Current", "%.1f", currentVelocity);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addLine("\nTune values in Dashboard!");
            telemetry.update();
        }
    }
}
/*{
    private static final double INCREMENT = 3.0;     // amount to step motor each CYCLE_MS cycle
    private static final int     CYCLE_MS = 20;       // period of each cycle
    private static final double   MAX_DIST = 136;     // Maximum that we can shoot
    private static final double   MIN_DIST = 0;     // Minimum that we can shoot.
    private static final double   FAV_DIST = 70;
    private double cps = 0.0;
    private static final double MIN_CPS = 0.0;
    //6000RPM/60 for RPS * 28 CPR for 1:1 goBilda motor = 2800
    private static final double MAX_CPS = (6000.0/60.0) * 28;
    private static final double CPS_INC = 20.0;

    double lastKp = RobotConstants.SH_PID.p;
    double lastKi = RobotConstants.SH_PID.i;
    double lastKd = RobotConstants.SH_PID.d;
    double lastKf = RobotConstants.SH_PID.f;
    
    public static PIDFCoefficients pidf = RobotConstants.SH_PID;

    private FtcDashboard dbd;

    private static Shooter shooter;

    private static final String TAG = "SJH_TSH";
    protected static final boolean VERBOSE = RobotConstants.logVerbose;

    private void doLogging()
    {
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("pos", shooter.getEncPos());
        packet.put("spd", shooter.getCurSpd());
        packet.put("cmd", cps);
        packet.put("dst", shooter.getDist());
        dbd.sendTelemetryPacket(packet);

        String shtStr = shooter.toString();
        if(VERBOSE){RobotLog.dd(TAG, shtStr);}
        //dashboard.displayText(0, shtStr);
    }

    @Override
    public void runOpMode()
    {
        initCommon(this);

        dbd = FtcDashboard.getInstance();
        dbd.setTelemetryTransmissionInterval(25);

        shooter = new Shooter(hardwareMap);
        shooter.init();

        shooter.setPIDF(pidf);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs)
        {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        double distance = FAV_DIST;

        // Wait for the start button
        while (!isStarted())
        {
            shooter.update();
            doLogging();
            sleep(CYCLE_MS);
        }
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive())
        {
            for (LynxModule module : allHubs)
            {
                module.clearBulkCache();
            }

            if (lastKp != pidf.p || lastKd != pidf.d || lastKi != pidf.i || lastKf != pidf.f)
            {
                shooter.setPIDF(pidf);

                lastKp = pidf.p;
                lastKi = pidf.i;
                lastKd = pidf.d;
                lastKf = pidf.f;
            }


            shooter.update();
            gpad1.update();

            boolean step_up    = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean step_down  = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean zeroize    = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
            boolean normal     = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            boolean shtInc     = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean shtDec     = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);

            if (shtInc) {cps += CPS_INC; cps = Math.min(cps, MAX_CPS);}
            else if (shtDec) {cps -=  CPS_INC; cps = Math.max(cps, MIN_CPS);}

            if(shtInc || shtDec) shooter.shootCps(cps);

            if (step_up && distance < MAX_DIST) {
                distance += INCREMENT;
                shooter.spinShooterW(distance);
                cps = shooter.getCmdSpd();
            }
            if (step_down && distance > MIN_DIST) {
                distance -= INCREMENT;
                shooter.spinShooterW(distance);
                cps = shooter.getCmdSpd();
            }
            if (zeroize)
            {
                shooter.stop();
                cps = shooter.getCmdSpd();
            }
            if (normal){
                distance = FAV_DIST;
                shooter.spinShooterW(distance);
                cps = shooter.getCmdSpd();
            }

            doLogging();

            sleep(CYCLE_MS);
        }
        shooter.stop();
    }
}
*/