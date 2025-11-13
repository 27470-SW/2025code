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
   private static float  delay;
   private static float  xOffset;
   private static int  autonDebug;

   private static int lastLoc;
   private static Field.Motif motif;
   private static Field.Num_shots numShots;
   private static int motifVar2;
   private static int numShotsVar2;

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
   public static int getLastLoc() { return lastLoc; }
   public static Field.Motif getMotif() {return motif; }
   public static Field.Num_shots getNumShots() {return numShots;}
   public static String getBotName()  { return botName; }
   public static String getAllianceColor() { return alliance; }
   public static int getStartPosition() { return startPos; }
   public static float getDelay() { return delay; }
   public static float getXOffset() { return xOffset; }
   public static int getEnableAutonDebug() { return autonDebug; }
   public void setBotName(String botName) { PreferenceMgr.botName = botName; }
   public void setAllianceColor(String allianceColor) { PreferenceMgr.alliance = allianceColor; }
   public void setStartPosition(int startPosition) { PreferenceMgr.startPos = startPosition; }
   public void setParkPosition(int parkPos)   { PreferenceMgr.parkPos = parkPos; }
   public void setDelay(float delay) { PreferenceMgr.delay =  delay; }
   public void setXOffset(float offset) { PreferenceMgr.xOffset =  offset; }
   public void setEnableAutonDebug(int debugEnable) { PreferenceMgr.autonDebug =  debugEnable; }
   public void setLastLoc(int LastLoc) { PreferenceMgr.lastLoc =  LastLoc; }
   public void setMotif(Field.Motif motif){
      PreferenceMgr.motif = motif;
      PreferenceMgr.motifVar2 = motif.ordinal();

   }
   public void setNumShots(Field.Num_shots numShot){
      PreferenceMgr.numShots = numShot;
      PreferenceMgr.numShotsVar2 = numShot.ordinal();

   }


   public PreferenceMgr()
   {
   }

   private static void setEnums(){

      try
      {
         motif = Field.Motif.values()[motifVar2];
      }
      catch(Exception e)
      {
         motif = Field.Motif.values()[0];
      }
      try
      {
         numShots = Field.Num_shots.values()[numShotsVar2];
      }
      catch(Exception e)
      {
         numShots = Field.Num_shots.values()[0];
      }

   }

   private static void readPrefs()
   {
      try{
      botName  = prefs.getString(CLUBNAME + ".botName", "B7252");
      alliance = prefs.getString(CLUBNAME + ".alliance", "RED");
      startPos = prefs.getInt(   CLUBNAME + ".startPos",1);
      parkPos = prefs.getInt(   CLUBNAME + ".parkPos",1);

      lastLoc = prefs.getInt(CLUBNAME + ".firstLoc", 1);
      delay    = prefs.getFloat( CLUBNAME + ".delay", 0.0f);
      xOffset  = prefs.getFloat( CLUBNAME + ".xOffset", 0.0f);
      autonDebug  = prefs.getInt( CLUBNAME + ".autonDebug", 1);

      motifVar2 =prefs.getInt(CLUBNAME + ".motif", 1);
      numShotsVar2 =prefs.getInt(CLUBNAME + ".numShots", 1);


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
         editor.putInt(CLUBNAME + ".firstLoc", lastLoc);
         editor.putInt(CLUBNAME + ".autonDebug", autonDebug);
         editor.putInt(CLUBNAME + ".motif", motifVar2);
         editor.putInt(CLUBNAME + ".numShots",numShotsVar2);
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
      RobotLog.dd(TAG, "delay:    %4.1f", delay);
      RobotLog.dd(TAG, "xOffset:  %4.1f", xOffset);
      RobotLog.dd(TAG, "firstLoc:  %d", lastLoc);
      RobotLog.dd(TAG, "AutonDebug:  %d", autonDebug);

      RobotLog.dd(TAG, "MotifVar : %d", motifVar2);
      RobotLog.dd(TAG, "numShotsVar: %d", numShotsVar2);
      RobotLog.dd(TAG, "motif: %s", motif);
      RobotLog.dd(TAG, "numShots: %s", numShots);


   }
   catch (Exception e){

      RobotLog.dd(TAG, "unable to log prefs or sum like that");
   }
   }
}