package org.firstinspires.ftc.teamcode.opModes;

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
    private static Field.StartPos startPosition;
    private static Field.Route autonStrategy;
    private static float delay;
    private static int curcuit;
    private static float xOffset;
    private static Field.AutonDebug autonDebugEnable;
    private static Field.Parks parkPos;
    private static Field.Wiffle_Pos firstLoc;
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
            startPosition = Field.StartPos.values()[PreferenceMgr.getStartPosition()];
        }
        catch(Exception e)
        {
            startPosition = Field.StartPos.values()[0];
        }

        try
        {
            autonStrategy = Field.Route.values()[PreferenceMgr.getAutonStrategy()];
        }
        catch(Exception e)
        {
            autonStrategy = Field.Route.values()[0];
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
            parkPos = Field.Parks.values()[PreferenceMgr.getParkPosition()];
        }
        catch(Exception e)
        {
            parkPos = Field.Parks.values()[0];
        }
        try
        {
            firstLoc = Field.Wiffle_Pos.values()[PreferenceMgr.getFirstLoc()];
        }
        catch(Exception e)
        {
            firstLoc = Field.Wiffle_Pos.values()[0];
        }

            stackSideHighwayToBackdrop = PreferenceMgr.getStackHighwayToBd();


            Highway1Var = PreferenceMgr.getHighway1();



            Highway2Var = PreferenceMgr.getHighway2();



            Highway3Var = PreferenceMgr.getHighway3();



            Highway12Var = PreferenceMgr.getHighway12();



            Highway22Var = PreferenceMgr.getHighway22();


          Highway32Var = PreferenceMgr.getHighway32();


            Pixel3Var = PreferenceMgr.getPixel3();



            Pixel2Var = PreferenceMgr.getPixel2();



            Pixel1Var = PreferenceMgr.getPixel1();




        delay         = PreferenceMgr.getDelay();
        curcuit         = PreferenceMgr.getCircuit();
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

    private static final boolean showBotMenu = true;
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
        FtcChoiceMenu<Field.StartPos> startPosMenu
            = new FtcChoiceMenu<>("START:", allianceMenu, this);
        //
        FtcValueMenu  delayMenu
            = new FtcValueMenu("Delay:",       startPosMenu,     this,
            0.0, 20.0, 1.0, delay, "%5.2f");
        //
        FtcValueMenu  xOffsetMenu
          = new FtcValueMenu("xOffset:",       delayMenu,     this,
            0.0, 12.0, 1.0, xOffset, "%5.2f");
        FtcChoiceMenu<Field.AutonDebug> autoDebugMenu
                = new FtcChoiceMenu<>("AUTON DEBUG:",   xOffsetMenu, this);
        FtcChoiceMenu<Field.Parks> parkPosMenu
                = new FtcChoiceMenu<>("Park Position:",   autoDebugMenu, this);
        FtcChoiceMenu<Field.Wiffle_Pos> firstLocationMenu
                = new FtcChoiceMenu<>("First Location:",   parkPosMenu, this);
        FtcChoiceMenu<Field.Parks> stackHighwayToBdMenu
                = new FtcChoiceMenu<>("Highway To Backdrop:",   firstLocationMenu, this);
        //
        FtcValueMenu  curcuitMenu
                = new FtcValueMenu("Amount of Curcuits:",       stackHighwayToBdMenu,     this,
                0.0, 3.0, 1.0, curcuit, "%5.2f");
        //
        FtcChoiceMenu<Field.Parks> Highway1
                = new FtcChoiceMenu<>("Highway1 ToPixel:",   curcuitMenu, this);
        FtcChoiceMenu<Field.Parks> Pixelstack1
                = new FtcChoiceMenu<>("Pixelstack1:",   Highway1, this);
        FtcChoiceMenu<Field.Parks> Highway12
                = new FtcChoiceMenu<>("Highway1 ToBackdrop:",   Pixelstack1, this);
        FtcChoiceMenu<Field.AutonDebug> sameCurcuitMenu
                = new FtcChoiceMenu<>("Unique Circuits:",   Highway12, this);



        FtcChoiceMenu<Field.Parks> Highway2
                = new FtcChoiceMenu<>("Highway2 ToPixel:",   null, this);
        FtcChoiceMenu<Field.Parks> Pixelstack2
                = new FtcChoiceMenu<>("Pixelstack2:",   Highway2, this);
        FtcChoiceMenu<Field.Parks> Highway22
                = new FtcChoiceMenu<>("Highway2 ToBackdrop:",   Pixelstack2, this);
        FtcChoiceMenu<Field.Parks> Highway3
                = new FtcChoiceMenu<>("Highway3 ToPixel:",   Highway22, this);
        FtcChoiceMenu<Field.Parks> Pixelstack3
                = new FtcChoiceMenu<>("Pixelstack3:",   Highway3, this);
        FtcChoiceMenu<Field.Parks> Highway32
                = new FtcChoiceMenu<>("Highway3 ToBackdrop:",   Pixelstack3, this);

        //
        // remember last saved settings and reorder the menu with last run settings as the defaults
        //

        for(RobotConstants.Chassis b : RobotConstants.Chassis.values())
        {
            botMenu.addChoice(b.toString(), b, b==bot, allianceMenu);
        }

        for(Field.Alliance a : Field.Alliance.values())
        {
            allianceMenu.addChoice(a.toString(), a, a==allianceColor, startPosMenu);
        }

        for(Field.StartPos p : Field.StartPos.values())
        {
            startPosMenu.addChoice(p.toString(), p, p==startPosition, p==Field.StartPos.START_SAMPLES?firstLocationMenu:curcuitMenu);
        }

        for(Field.Wiffle_Pos f : Field.Wiffle_Pos.values())
        {
            firstLocationMenu.addChoice(f.toString(), f, f == firstLoc, stackHighwayToBdMenu);
        }
        for(Field.Parks g : Field.Parks.values())
        {
            stackHighwayToBdMenu.addChoice(g.toString(), g, g == stackSideHighwayToBackdrop, curcuitMenu);
        }
        curcuitMenu.setChildMenu(null);
        for(Field.Parks g : Field.Parks.values())
        {
            Highway1.addChoice(g.toString(), g, g == Highway1Var, Pixelstack1);
        }
        for(Field.Parks h : Field.Parks.values())
        {
            Pixelstack1.addChoice(h.toString(), h, h == Pixel1Var, Highway12);
        }
        for(Field.Parks i : Field.Parks.values())
        {
            Highway12.addChoice(i.toString(), i, i == Highway12Var, null);
        }
        for(Field.AutonDebug p : Field.AutonDebug.values())
        {
            sameCurcuitMenu.addChoice(p.toString(), p, p == uniqecircuits, null);
        }
        for(Field.Parks j : Field.Parks.values())
        {
            Highway2.addChoice(j.toString(), j, j == Highway2Var, Pixelstack2);
        }
        for(Field.Parks k : Field.Parks.values())
        {
            Pixelstack2.addChoice(k.toString(), k, k == Pixel2Var, Highway22);
        }
        for(Field.Parks l : Field.Parks.values())
        {
            Highway22.addChoice(l.toString(), l, l == Highway22Var, Highway3);
        }
        for(Field.Parks m : Field.Parks.values())
        {
            Highway3.addChoice(m.toString(), m, m == Highway3Var, Pixelstack3);
        }
        for(Field.Parks n : Field.Parks.values())
        {
            Pixelstack3.addChoice(n.toString(), n, n == Pixel3Var, Highway32);
        }
        for(Field.Parks o : Field.Parks.values())
        {
            Highway32.addChoice(o.toString(), o, o == Highway32Var, parkPosMenu);
        }

        for(Field.Parks e : Field.Parks.values())
        {
            parkPosMenu.addChoice(e.toString(), e, e == parkPos, delayMenu);
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

        curcuit =(int)curcuitMenu.getCurrentValue();
        if(curcuit > 0){
            FtcMenu.walkMenuTree(Highway1, this);

        }
        if(curcuit > 1){
            FtcMenu.walkMenuTree(sameCurcuitMenu, this);


            uniqecircuits = sameCurcuitMenu.getCurrentChoiceObject();

            if (uniqecircuits == Field.AutonDebug.ENABLE){
                FtcMenu.walkMenuTree(Highway2, this);
                Highway2Var = Highway2.getCurrentChoiceObject();
                Highway3Var = Highway3.getCurrentChoiceObject();
                Highway22Var = Highway22.getCurrentChoiceObject();
                Highway32Var = Highway3.getCurrentChoiceObject();
                Highway12Var = Highway12.getCurrentChoiceObject();
                Highway1Var = Highway1.getCurrentChoiceObject();
            }
            else {
                Highway12Var = Highway12.getCurrentChoiceObject();
                Highway1Var = Highway1.getCurrentChoiceObject();
                Highway2Var = Highway1Var;
                Highway3Var = Highway1Var;
                Highway22Var = Highway12Var;
                Highway32Var = Highway12Var;

                Pixelstack2.removeAllChoices();
                Pixelstack3.removeAllChoices();
                for (Field.Parks k : Field.Parks.values()) {
                    Pixelstack2.addChoice(k.toString(), k, k == Pixel2Var, Pixelstack3);
                }
                for (Field.Parks n : Field.Parks.values()) {
                    Pixelstack3.addChoice(n.toString(), n, n == Pixel3Var, parkPosMenu);
                }
                FtcMenu.walkMenuTree(Pixelstack2, this);


            }
        }else{
            FtcMenu.walkMenuTree(parkPosMenu, this);
        }
        Highway12Var = Highway12.getCurrentChoiceObject();
        Highway1Var = Highway1.getCurrentChoiceObject();
        Pixel1Var = Pixelstack1.getCurrentChoiceObject();
        Pixel2Var = Pixelstack2.getCurrentChoiceObject();
        Pixel3Var = Pixelstack3.getCurrentChoiceObject();
        startPosition = startPosMenu.getCurrentChoiceObject();
        allianceColor = allianceMenu.getCurrentChoiceObject();
        delay = (float)delayMenu.getCurrentValue();
        xOffset = (float)xOffsetMenu.getCurrentValue();
        autonDebugEnable = autoDebugMenu.getCurrentChoiceObject();
        parkPos = parkPosMenu.getCurrentChoiceObject();
        stackSideHighwayToBackdrop = stackHighwayToBdMenu.getCurrentChoiceObject();



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
        prfMgr.setFirstLoc(firstLoc.ordinal());
        prfMgr.setStackHighwayToBackdrop(stackSideHighwayToBackdrop.ordinal());
        prfMgr.setCircuit(curcuit);
        prfMgr.setHighway1(Highway1Var.ordinal());
        prfMgr.setHighway2(Highway2Var.ordinal());
        prfMgr.setHighway3(Highway3Var.ordinal());
        prfMgr.setPixel1(Pixel1Var.ordinal());
        prfMgr.setPixel2(Pixel2Var.ordinal());
        prfMgr.setPixel3(Pixel3Var.ordinal());
        prfMgr.setHighway12(Highway12Var.ordinal());
        prfMgr.setHighway22(Highway22Var.ordinal());
        prfMgr.setHighway32(Highway32Var.ordinal());

        //write the options to sharedpreferences
        PreferenceMgr.writePrefs();

        //read them back to ensure they were written
        getPrefs();

        RobotLog.dd(TAG, "Returned Config Values:");
        printConfigToLog();

        dashboard.displayText(lnum++, "Bot:      " + bot);
        dashboard.displayText(lnum++, "Alliance: " + allianceColor);
        dashboard.displayText(lnum++, "Start:    " + startPosition);
        dashboard.displayText(lnum++, "Delay:    " + delay);
        dashboard.displayText(lnum++, "xOffset:  " + xOffset);
        dashboard.displayText(lnum++, "Auton Debug:  " + autonDebugEnable);
        dashboard.displayText(lnum++, "Park Position:  " + parkPos);
        dashboard.displayText(lnum++, "First Location:  " + firstLoc + " : " + stackSideHighwayToBackdrop );
        dashboard.displayText(lnum++, "Curcuit:    " + curcuit);
        dashboard.displayText(lnum++, "Curcuit 1 " + Highway1Var +"," + Pixel1Var +"," + Highway12Var);
        dashboard.displayText(lnum++, "Curcuit 2 " + Highway2Var +"," + Pixel2Var +"," + Highway22Var);
        dashboard.displayText(lnum++, "Curcuit 3 " + Highway3Var +"," + Pixel3Var +"," + Highway32Var);
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
        RobotLog.dd(TAG, "unique circuits:  %s", uniqecircuits);
        RobotLog.dd(TAG, "First Location:  %s", firstLoc);
        RobotLog.dd(TAG, "stackHighwayToBd:  %s", stackSideHighwayToBackdrop);
        RobotLog.dd(TAG, "circuit:    %d", curcuit);
        RobotLog.dd(TAG, "Highway1:  %s", Highway1Var);
        RobotLog.dd(TAG, "Highway2:  %s", Highway2Var);
        RobotLog.dd(TAG, "Highway3:  %s", Highway3Var);
        RobotLog.dd(TAG, "Pixel1:  %s", Pixel1Var);
        RobotLog.dd(TAG, "Pixel2:  %s", Pixel2Var);
        RobotLog.dd(TAG, "Pixel3:  %s", Pixel3Var);
        RobotLog.dd(TAG, "Highway1 2:  %s", Highway12Var);
        RobotLog.dd(TAG, "Highway2 2:  %s", Highway22Var);
        RobotLog.dd(TAG, "Highway3 2:  %s", Highway32Var);
    }

}
