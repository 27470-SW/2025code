package org.firstinspires.ftc.teamcode.util;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.opModes.AutonConfig;

@SuppressWarnings("unused")
public class PreferenceMgr
{
   private static final String TAG = "SJH_PRF";
   private static SharedPreferences prefs;
   private static final String CLUBNAME = "Messmore";

   private static String botName;
   private static String alliance;
   private static int    startPos;
   private static int    parkPos;
   private static Field.Parks pixel1;
   private static Field.Parks pixel2;
   private static Field.Parks pixel3;
   private static Field.Parks Highway1;
   private static Field.Parks Highway2;
   private static Field.Parks Highway3;
   private static Field.Parks Highway12;
   private static Field.Parks Highway22;
   private static Field.Parks Highway32;
   private static Field.Parks stackHighwayToBackdrop;
   private static int autonStrgy;
   private static float  delay;
   private static int  circuit = -1;
   private static float  xOffset;
   private static int  autonDebug;
   private static int firstLoc;
   private static Field.Motif motif;
   private static Field.Num_shots numShots;
   private static int stackHighwayToBackdropVar;
   private static int Highway1Var;
   private static int Highway2Var;
   private static int Highway3Var;
   private static int Pixel1Var;
   private static int  Pixel2Var;
   private static int Pixel3Var;
   private static int Highway12Var;
   private static int  Highway22Var;
   private static int Highway32Var;

   static
   {
      RobotLog.dd(TAG, "Init static block");
      RobotLog.dd(TAG, "applicationID=%s", AppUtil.getInstance().getApplicationId());
      prefs = PreferenceManager.getDefaultSharedPreferences(AppUtil.getInstance().getApplication());
      readPrefs();
      writePrefs(); //This will write defaults if prefs were empty at init
   }

   public static String getClubName() { return CLUBNAME; }
   public static int getParkPosition() { return parkPos; }
   public static int getFirstLoc() { return firstLoc; }
   public static Field.Motif getMotif() {return motif; }
   public static Field.Num_shots getNumShots() {return numShots;}
   public static Field.Parks getStackHighwayToBd() { return stackHighwayToBackdrop; }
   public static Field.Parks getPixel1() { return pixel1; }
   public static Field.Parks getPixel2() { return pixel2; }
   public static Field.Parks getPixel3() { return pixel3; }
   public static Field.Parks getHighway1() { return Highway1; }
   public static Field.Parks getHighway2() { return Highway2; }
   public static Field.Parks getHighway3() { return Highway3; }
   public static Field.Parks getHighway12() { return Highway12; }
   public static Field.Parks getHighway22() { return Highway22; }
   public static Field.Parks getHighway32() { return Highway32; }
   public static String getBotName()  { return botName; }
   public static String getAllianceColor() { return alliance; }
   public static int getStartPosition() { return startPos; }
   public static int getAutonStrategy() { return autonStrgy; }
   public static float getDelay() { return delay; }
   public static int getCircuit() { return circuit; }
   public static float getXOffset() { return xOffset; }
   public static int getEnableAutonDebug() { return autonDebug; }
   public void setBotName(String botName) { PreferenceMgr.botName = botName; }
   public void setAllianceColor(String allianceColor) { PreferenceMgr.alliance = allianceColor; }
   public void setStartPosition(int startPosition) { PreferenceMgr.startPos = startPosition; }
   public void setParkPosition(int parkPos)   { PreferenceMgr.parkPos = parkPos; }
   public void setDelay(float delay) { PreferenceMgr.delay =  delay; }
   public void setCircuit(int circuit) { PreferenceMgr.circuit =  circuit; }
   public void setXOffset(float offset) { PreferenceMgr.xOffset =  offset; }
   public void setEnableAutonDebug(int debugEnable) { PreferenceMgr.autonDebug =  debugEnable; }
   public void setHighway1(int Highway1) { PreferenceMgr.Highway1Var =  Highway1; RobotLog.dd(TAG, "PRM Highway1:  %d", Highway1Var);}
   public void setHighway2(int Highway2) { PreferenceMgr.Highway2Var =  Highway2; }
   public void setHighway3(int Highway3) { PreferenceMgr.Highway3Var =  Highway3; }
   public void setPixel1(int Pixel1) { PreferenceMgr.Pixel1Var =  Pixel1; }
   public void setPixel2(int Pixel2) { PreferenceMgr.Pixel2Var =  Pixel2; }
   public void setPixel3(int Pixel3) { PreferenceMgr.Pixel3Var =  Pixel3; }
   public void setHighway12(int Highway12) { PreferenceMgr.Highway12Var =  Highway12; }
   public void setHighway22(int Highway23) { PreferenceMgr.Highway22Var =  Highway23; }
   public void setHighway32(int Highway32) { PreferenceMgr.Highway32Var =  Highway32; }
   public void setFirstLoc(int FirstLoc) { PreferenceMgr.firstLoc =  FirstLoc; }
   public void setStackHighwayToBackdrop(int stackHighway) { PreferenceMgr.stackHighwayToBackdropVar =  stackHighway; }


   public PreferenceMgr()
   {
   }

   private static void setEnums(){

      try
      {
         Highway1 = Field.Parks.values()[Highway1Var];
      }
      catch(Exception e)
      {
         Highway1 = Field.Parks.values()[0];
      }
      try
      {
         Highway2 = Field.Parks.values()[Highway2Var];
      }
      catch(Exception e)
      {
         Highway2 = Field.Parks.values()[0];
      }
      try
      {
         Highway3 = Field.Parks.values()[Highway3Var];
      }
      catch(Exception e)
      {
         Highway3 = Field.Parks.values()[0];
      }
      try
      {
         Highway12 = Field.Parks.values()[Highway12Var];
      }
      catch(Exception e)
      {
         Highway12 = Field.Parks.values()[0];
      }
      try
      {
         Highway22 = Field.Parks.values()[Highway22Var];
      }
      catch(Exception e)
      {
         Highway22 = Field.Parks.values()[0];
      }
      try
      {
         Highway32 = Field.Parks.values()[Highway32Var];
      }
      catch(Exception e)
      {
         Highway32 = Field.Parks.values()[0];
      }
      try
      {
         pixel3 = Field.Parks.values()[Pixel3Var];
      }
      catch(Exception e)
      {
         pixel3 = Field.Parks.values()[0];
      }
      try
      {
         pixel2 = Field.Parks.values()[Pixel2Var];
      }
      catch(Exception e)
      {
         pixel2 = Field.Parks.values()[0];
      }
      try
      {
         pixel1 = Field.Parks.values()[Pixel1Var];
      }
      catch(Exception e)
      {
         pixel1 = Field.Parks.values()[0];
      }
      try
      {
            stackHighwayToBackdrop = Field.Parks.values()[stackHighwayToBackdropVar];
      }
      catch(Exception e)
      {
         stackHighwayToBackdrop = Field.Parks.values()[0];
      }

   }

   private static void readPrefs()
   {
      try{
      botName  = prefs.getString(CLUBNAME + ".botName", "B7252");
      alliance = prefs.getString(CLUBNAME + ".alliance", "RED");
      startPos = prefs.getInt(   CLUBNAME + ".startPos",1);
      parkPos = prefs.getInt(   CLUBNAME + ".parkPos",1);
      autonStrgy = prefs.getInt(CLUBNAME + ".autoStrag", 1);
      firstLoc = prefs.getInt(CLUBNAME + ".firstLoc", 1);
      delay    = prefs.getFloat( CLUBNAME + ".delay", 0.0f);
      xOffset  = prefs.getFloat( CLUBNAME + ".xOffset", 0.0f);
      autonDebug  = prefs.getInt( CLUBNAME + ".autonDebug", 1);
      circuit    = prefs.getInt( CLUBNAME + ".Circuit", 1);
      Pixel1Var    = prefs.getInt(CLUBNAME + ".Pixel1", 1);
      Pixel2Var    = prefs.getInt(CLUBNAME + ".Pixel2", 1);
      Pixel3Var    = prefs.getInt(CLUBNAME + ".Pixel3", 1);
      Highway1Var    = prefs.getInt(CLUBNAME + ".Highway1", 1);
      Highway2Var    = prefs.getInt(CLUBNAME + ".Highway2", 1);
      Highway3Var    = prefs.getInt(CLUBNAME + ".Highway3", 1);
      Highway12Var    = prefs.getInt(CLUBNAME + ".Highway12", 1);
      Highway22Var    = prefs.getInt(CLUBNAME + ".Highway22", 1);
      Highway32Var    = prefs.getInt(CLUBNAME + ".Highway32", 1);
      stackHighwayToBackdropVar = prefs.getInt(CLUBNAME + ".stackHighway", 1);


   }catch(Exception e){
         RobotLog.dd(TAG, "unable to read prefs or sum like that");
   }

   }

   public static void writePrefs()
   {
      try {
         //write the options to sharedpreferences
         SharedPreferences.Editor editor = prefs.edit();
         editor.putString(CLUBNAME + ".botName", botName);
         editor.putString(CLUBNAME + ".alliance", alliance);
         editor.putInt(CLUBNAME + ".startPos", startPos);
         editor.putInt(CLUBNAME + ".parkPos", parkPos);
         editor.putFloat(CLUBNAME + ".delay", delay);
         editor.putFloat(CLUBNAME + ".xOffset", xOffset);
         editor.putInt(CLUBNAME + ".firstLoc", firstLoc);
         editor.putInt(CLUBNAME + ".autonDebug", autonDebug);
         editor.putInt(CLUBNAME + ".Pixel1", Pixel1Var);
         editor.putInt(CLUBNAME + ".Pixel2", Pixel2Var);
         editor.putInt(CLUBNAME + ".Pixel3", Pixel3Var);
         editor.putInt(CLUBNAME + ".Highway1", Highway1Var);
         editor.putInt(CLUBNAME + ".Highway2", Highway2Var);
         editor.putInt(CLUBNAME + ".Highway3", Highway3Var);
         editor.putInt(CLUBNAME + ".Highway12", Highway12Var);
         editor.putInt(CLUBNAME + ".Highway22", Highway22Var);
         editor.putInt(CLUBNAME + ".Highway32", Highway32Var);
         editor.putFloat(CLUBNAME + ".Circuits", circuit);
         editor.putInt(CLUBNAME + ".stackHighway", stackHighwayToBackdropVar);
         editor.apply();

         prefs = PreferenceManager.getDefaultSharedPreferences(AppUtil.getInstance().getApplication());
      }catch (Exception e){
         RobotLog.dd(TAG, "unable to write prefs or sum like that");
      }
      setEnums();
   }

   public static void logPrefs()
   {try{
      RobotLog.dd(TAG, "Default Config Values:");
      RobotLog.dd(TAG, "Club:     %s", CLUBNAME);
      RobotLog.dd(TAG, "Bot:      %s", botName);
      RobotLog.dd(TAG, "Alliance: %s", alliance);
      RobotLog.dd(TAG, "startPos: %d", startPos);
      RobotLog.dd(TAG, "autonStrategy:  %d", autonStrgy);
      RobotLog.dd(TAG, "delay:    %4.1f", delay);
      RobotLog.dd(TAG, "xOffset:  %4.1f", xOffset);
      RobotLog.dd(TAG, "firstLoc:  %d", firstLoc);
      RobotLog.dd(TAG, "AutonDebug:  %d", autonDebug);
      RobotLog.dd(TAG, "Pixel1: %d", Pixel1Var);
      RobotLog.dd(TAG, "Pixel2: %d", Pixel2Var);
      RobotLog.dd(TAG, "Pixel3: %d", Pixel3Var);
      RobotLog.dd(TAG, "Highway1: %d", Highway1Var);
      RobotLog.dd(TAG, "Highway2: %d", Highway2Var);
      RobotLog.dd(TAG, "Highway3: %d", Highway3Var);
      RobotLog.dd(TAG, "Highway12: %d", Highway12Var);
      RobotLog.dd(TAG, "Highway22: %d", Highway22Var);
      RobotLog.dd(TAG, "Highway32: %d", Highway32Var);
      RobotLog.dd(TAG, "Pixel1: %s", pixel1);
      RobotLog.dd(TAG, "Pixel2: %s", pixel2);
      RobotLog.dd(TAG, "Pixel3: %s", pixel3);
      RobotLog.dd(TAG, "Highway1: %s", Highway1);
      RobotLog.dd(TAG, "Highway2: %s", Highway2);
      RobotLog.dd(TAG, "Highway3: %s", Highway3);
      RobotLog.dd(TAG, "Highway12: %s", Highway12);
      RobotLog.dd(TAG, "Highway22: %s", Highway22);
      RobotLog.dd(TAG, "Highway32: %s", Highway32);
      RobotLog.dd(TAG, "circuits:    %d", circuit);
      RobotLog.dd(TAG, "stackHighway: %d", stackHighwayToBackdropVar);
      RobotLog.dd(TAG, "stackHighway: %s", stackHighwayToBackdrop);

   }
   catch (Exception e){

      RobotLog.dd(TAG, "unable to log prefs or sum like that");
   }
   }
}