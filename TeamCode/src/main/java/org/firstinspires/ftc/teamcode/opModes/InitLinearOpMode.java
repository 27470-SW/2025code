package org.firstinspires.ftc.teamcode.opModes;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.field.PositionOption;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.PreferenceMgr;

import org.firstinspires.ftc.teamcode.util.HalDashboard;

public abstract class InitLinearOpMode extends LinearOpMode
{
    private static final String TAG = "SJH_ILO";
    protected static final String robotName;
    protected static final RobotConstants.Chassis chas;
    static {
        RobotLog.dd(TAG, "Init static block");
        PreferenceMgr.logPrefs();
        robotName = PreferenceMgr.getBotName();
        RobotLog.dd(TAG, "robotname: " + robotName);

        RobotConstants.Chassis tmpChas = RobotConstants.Chassis.B7252;
        try
        {
            tmpChas = RobotConstants.Chassis.valueOf(robotName);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "Robotname %s invalid. Defaulting to %s", robotName, tmpChas);
        }
        chas = tmpChas;

        RobotConstants.init(chas);
    }

    @OnCreate
    public static void onCreate(Context c)
    {
        RobotLog.dd(TAG, "onCreate called");
    }

    @OnCreateEventLoop
    public static void onCreateEventLoop(Context c, FtcEventLoop l)
    {
        RobotLog.dd(TAG, "onCreateEventLoop called");
    }

    protected static Field.Alliance alliance;
    protected static Field.Start_Pos startPos;
    protected static Field.Park_Pos parkPos;
    protected CommonUtil cmu = CommonUtil.getInstance();
    protected static DataLogger dl;
    protected static boolean logData = false;
    protected HalDashboard dashboard;
    protected ManagedGamepad gpad1;
    protected ManagedGamepad gpad2;

    protected boolean useOpenCv  = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardwareMap.logDevices();
    }

    //public void runOpMode(boolean );

    public void initCommon(LinearOpMode op)
    {
        cmu.init(op, useOpenCv, alliance, startPos);
        dl = cmu.getDataLogger();
        dashboard = cmu.getDashboard();
        gpad1 = new ManagedGamepad(gamepad1);
        gpad2 = new ManagedGamepad(gamepad2);
    }

    @SuppressWarnings("unused")
    public void cleanup()
    {
        if(dl != null) dl.closeDataLogger();
        if(dashboard != null) dashboard.clearDisplay();
    }

    public void doAuton(Route route){

    }
}
