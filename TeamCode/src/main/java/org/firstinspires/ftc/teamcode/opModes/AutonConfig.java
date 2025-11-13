package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.field.Field.Park_Pos.OUTSIDEPRIMARYPARK2RED;
import static org.firstinspires.ftc.teamcode.field.Field.Start_Pos.START_FAR_RED6;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.util.PreferenceMgr;

import org.firstinspires.ftc.teamcode.util.FtcChoiceMenu;
import org.firstinspires.ftc.teamcode.util.FtcMenu;
import org.firstinspires.ftc.teamcode.util.FtcValueMenu;

@Autonomous(name = "Auton Config", group = "0")
public class AutonConfig extends InitLinearOpMode implements FtcMenu.MenuButtons {

    private static final String TAG = "Auton Menu";

    //The autonomous menu settings using sharedpreferences
    private final PreferenceMgr prfMgr = new PreferenceMgr();
    private final static String club;
    private static RobotConstants.Chassis bot;
    private static Field.Alliance allianceColor;
    private static Field.Start_Pos startPosition;
    private static Field.Route autonStrategy;
    private static float delay;
    private static int curcuit;
    private static float xOffset;
    private static Field.AutonDebug autonDebugEnable;
    private static Field.Park_Pos parkPos;
    private static Field.Wiffle_Pos lastLoc;
    private static Field.Parks stackSideHighwayToBackdrop;
    private static Field.Parks Highway1Var;
    private static Field.Parks Pixel1Var;
    private static Field.Parks Highway12Var;
    private static Field.Parks Highway22Var;
    private static Field.Parks Highway32Var;
    private static Field.Parks Pixel2Var;
    private static Field.Parks Pixel3Var;
    private static Field.Parks Highway2Var;
    private static Field.Parks Highway3Var;
    private static Field.AutonDebug uniqecircuits;

    private static Field.Motif motif;
    private static Field.Num_shots numShots;
    private static int motifVar2;
    private static int numShotsVar2;


    static
    {
        club = PreferenceMgr.getClubName();
        getPrefs();
    }
    private int lnum = 1;

    public AutonConfig()
    {
        RobotLog.dd(TAG, "AutonConfig ctor");
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this);
        dashboard.displayText(0, "Starting Menu System");
        setup();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive())
        {
            idle();
        }
    }


    private void setup()
    {
        dashboard.displayText(0, "INITIALIZING - Please wait for Menu");
        doMenus();
        dashboard.displayText(0, "COMPLETE - Settings Written");
    }

    private static void getPrefs()
    {
        try
        {
            bot = RobotConstants.Chassis.valueOf(PreferenceMgr.getBotName());
        }
        catch (Exception e)
        {
            bot = RobotConstants.Chassis.values()[0];
        }
        try
        {
            allianceColor = Field.Alliance.valueOf(PreferenceMgr.getAllianceColor());
        }
        catch(Exception e)
        {
            allianceColor = Field.Alliance.values()[0];
        }

        try
        {
            startPosition = Field.Start_Pos.values()[PreferenceMgr.getStartPosition()];
        }
        catch(Exception e)
        {
            startPosition = Field.Start_Pos.values()[0];
        }

        try
        {
            motif = PreferenceMgr.getMotif();
        }
        catch(Exception e)
        {
            motif = Field.Motif.values()[0];
        }
        try
        {
            numShots = PreferenceMgr.getNumShots();
        }
        catch(Exception e)
        {
            numShots = Field.Num_shots.values()[0];
        }
        try
        {
            autonDebugEnable = Field.AutonDebug.values()[PreferenceMgr.getEnableAutonDebug()];
        }
        catch(Exception e)
        {
            autonDebugEnable = Field.AutonDebug.values()[0];
        }
        try
        {
            parkPos = Field.Park_Pos.values()[PreferenceMgr.getParkPosition()];
        }
        catch(Exception e)
        {
            parkPos = Field.Park_Pos.values()[0];
        }
        try
        {
            lastLoc = Field.Wiffle_Pos.values()[PreferenceMgr.getLastLoc()];
        }
        catch(Exception e)
        {
            lastLoc = Field.Wiffle_Pos.values()[0];
        }


        delay         = PreferenceMgr.getDelay();
        xOffset       = PreferenceMgr.getXOffset();

        PreferenceMgr.logPrefs();
    }

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()   { return gamepad1.dpad_up;}

    @Override
    public boolean isMenuAltUpButton()
    {
        return gamepad1.left_bumper;
    }

    @Override
    public boolean isMenuDownButton() { return gamepad1.dpad_down; }

    @Override
    public boolean isMenuAltDownButton()
    {
        return gamepad1.right_bumper;
    }

    @Override
    public boolean isMenuEnterButton() { return gamepad1.a; }

    @Override
    public boolean isMenuBackButton() { return gamepad1.dpad_left; }

    private static final boolean showBotMenu = false;
    private void doMenus()
    {
        FtcChoiceMenu<RobotConstants.Chassis> botMenu
            = new FtcChoiceMenu<>("Bot:",      null,         this);
        FtcMenu allianceParent = null;
        if(showBotMenu) allianceParent = botMenu;
        FtcChoiceMenu<Field.Alliance> allianceMenu
            = new FtcChoiceMenu<>("Alliance:", allianceParent,      this);
        FtcMenu topMenu = botMenu;
        if(!showBotMenu) topMenu = allianceMenu;

        FtcChoiceMenu<Field.Start_Pos> startPosMenuRed
            = new FtcChoiceMenu<>("START:", allianceMenu, this);
        FtcChoiceMenu<Field.Park_Pos> parkPosMenuRed
                = new FtcChoiceMenu<>("Park Position:",   startPosMenuRed, this);

        FtcChoiceMenu<Field.Start_Pos> startPosMenuBlue
                = new FtcChoiceMenu<>("START:", allianceMenu, this);
        FtcChoiceMenu<Field.Park_Pos> parkPosMenuBlue
                = new FtcChoiceMenu<>("Park Position:",   startPosMenuBlue, this);

        FtcChoiceMenu<Field.Num_shots> numShotMenu
                = new FtcChoiceMenu<>("Number of 3-ball shots:",   parkPosMenuRed, this);
        FtcChoiceMenu<Field.Wiffle_Pos> lastLocationMenu
                = new FtcChoiceMenu<>("Last Location:",   numShotMenu, this);

        FtcValueMenu  delayMenu
            = new FtcValueMenu("Delay:",       lastLocationMenu,     this,
            0.0, 20.0, 1.0, delay, "%5.2f");
        //
        FtcValueMenu  xOffsetMenu
          = new FtcValueMenu("xOffset:",       delayMenu,     this,
            0.0, 12.0, 1.0, xOffset, "%5.2f");
        FtcChoiceMenu<Field.AutonDebug> autoDebugMenu
                = new FtcChoiceMenu<>("AUTON DEBUG:",   xOffsetMenu, this);

        //
        // remember last saved settings and reorder the menu with last run settings as the defaults
        //

        for(RobotConstants.Chassis b : RobotConstants.Chassis.values())
        {
            botMenu.addChoice(b.toString(), b, b==bot, allianceMenu);
        }

        for(Field.Alliance a : Field.Alliance.values())
        {
            allianceMenu.addChoice(a.toString(), a, a==allianceColor, a== Field.Alliance.RED?startPosMenuRed:startPosMenuBlue);
        }

        for(Field.Start_Pos p : Field.Start_Pos.values())
        {
            if(p.ordinal() <= START_FAR_RED6.ordinal()) {
                startPosMenuRed.addChoice(p.toString(), p, p == startPosition, parkPosMenuRed);
            }
        }
        for(Field.Start_Pos p : Field.Start_Pos.values())
        {
            if(p.ordinal() > START_FAR_RED6.ordinal()) {
                startPosMenuBlue.addChoice(p.toString(), p, p == startPosition, parkPosMenuBlue);
            }
        }


        for(Field.Park_Pos g : Field.Park_Pos.values())
        {
            if(g.ordinal() <= OUTSIDEPRIMARYPARK2RED.ordinal()) {
                parkPosMenuRed.addChoice(g.toString(), g, g == parkPos, numShotMenu);
            }
        }
        for(Field.Park_Pos g : Field.Park_Pos.values())
        {
            if(g.ordinal() > OUTSIDEPRIMARYPARK2RED.ordinal()) {
                parkPosMenuBlue.addChoice(g.toString(), g, g == parkPos, numShotMenu);
            }
        }

        for(Field.Num_shots g : Field.Num_shots.values()){
            numShotMenu.addChoice(g.toString(), g, g==numShots, lastLocationMenu);
        }
        for(Field.Wiffle_Pos f : Field.Wiffle_Pos.values())
        {
            lastLocationMenu.addChoice(f.toString(), f, f == lastLoc, delayMenu);
        }


        delayMenu.setChildMenu(xOffsetMenu);
        xOffsetMenu.setChildMenu(autoDebugMenu);

        for(Field.AutonDebug d : Field.AutonDebug.values())
        {
            autoDebugMenu.addChoice(d.toString(), d, d== autonDebugEnable, null);
        }

        //
        // Walk the menu tree starting with the strategy menu as the root
        // menu and get user choices.
        //



        FtcMenu.walkMenuTree(topMenu, this);

        //
        // Set choices variables.
        //

        if(showBotMenu)
        {
            bot = botMenu.getCurrentChoiceObject();
        }

        allianceColor = allianceMenu.getCurrentChoiceObject();

        if(allianceColor == Field.Alliance.RED) {
            startPosition = startPosMenuRed.getCurrentChoiceObject();
        }else {
            startPosition = startPosMenuBlue.getCurrentChoiceObject();
        }

        parkPos = allianceColor == Field.Alliance.RED?parkPosMenuRed.getCurrentChoiceObject():parkPosMenuBlue.getCurrentChoiceObject();

        delay = (float)delayMenu.getCurrentValue();
        xOffset = (float)xOffsetMenu.getCurrentValue();
        autonDebugEnable = autoDebugMenu.getCurrentChoiceObject();
        numShots = numShotMenu.getCurrentChoiceObject();
        lastLoc = lastLocationMenu.getCurrentChoiceObject();



        //
        // Set choices variables.
        //

        RobotLog.dd(TAG, "Writing Config Values:");
        printConfigToLog();

        prfMgr.setBotName(bot.toString());
        prfMgr.setStartPosition(startPosition.ordinal());
        prfMgr.setParkPosition(parkPos.ordinal());
        prfMgr.setAllianceColor(allianceColor.toString());
        prfMgr.setDelay(delay);
        prfMgr.setXOffset(xOffset);
        prfMgr.setEnableAutonDebug(autonDebugEnable.ordinal());
        prfMgr.setLastLoc(lastLoc.ordinal());
        prfMgr.setNumShots(numShots);



        //write the options to sharedpreferences
        PreferenceMgr.writePrefs();

        //read them back to ensure they were written
        getPrefs();

        RobotLog.dd(TAG, "Returned Config Values:");
        printConfigToLog();

        dashboard.displayText(lnum++, "Bot:      " + bot);
        dashboard.displayText(lnum++, "Alliance: " + allianceColor);
        dashboard.displayText(lnum++, "Start:    " + startPosition);
        dashboard.displayText(lnum++, "Num Shots:" + numShots);
        dashboard.displayText(lnum++, "Last Location:  " + lastLoc);
        dashboard.displayText(lnum++, "Park Position:  " + parkPos);
        dashboard.displayText(lnum++, "Delay:    " + delay);
        dashboard.displayText(lnum++, "xOffset:  " + xOffset);
        dashboard.displayText(lnum++, "Auton Debug:  " + autonDebugEnable);
    }

    public void printConfigToLog()
    {
        RobotLog.dd(TAG, "Club:     %s", club);
        RobotLog.dd(TAG, "Bot:      %s", bot);
        RobotLog.dd(TAG, "Alliance: %s", allianceColor);
        RobotLog.dd(TAG, "startPos: %s", startPosition);
        RobotLog.dd(TAG, "parkPos:  %s", parkPos);
        RobotLog.dd(TAG, "delay:    %4.1f", delay);
        RobotLog.dd(TAG, "xOffset:  %4.1f", xOffset);
        RobotLog.dd(TAG, "auton Debug:  %s", autonDebugEnable);
        RobotLog.dd(TAG, "Last Location:  %s", lastLoc);
        RobotLog.dd(TAG, "Num Shots:  %s", numShots);

    }

}
