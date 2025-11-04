package com.example.meepmeeptesting;

import java.util.ArrayList;
import java.util.List;


@SuppressWarnings("unused")
public abstract class Field
{
    public Field(String assetName)
    {
        this.assetName = assetName;
        setImageNames();
        setImageLocations();
    }

    private final String         assetName;
            List<String>         trackableNames   = new ArrayList<>();

    public enum Alliance {BLUE, RED}

    //  X axis parallel to red  alliance wall point toward    blue alliance
    //  Y axis parallel to blue alliance wall point away from red  alliance

    //The descriptions say the field is 12'x12', but our
    //practice field is actually slightly smaller at 141"
    private static final float X_WIDTH = 141.0f;
    private static final float Y_WIDTH = 141.0f;
    /* package */ static final float N_WALL_Y = Y_WIDTH/2.0f;
    /* package */ static final float E_WALL_X = X_WIDTH/2.0f;
    /* package */ static final float S_WALL_Y = -N_WALL_Y;
    /* package */ static final float W_WALL_X = -E_WALL_X;

    public static final float tileWidth        = 23.5f;
    public static final float oneTile          = tileWidth;
    public static final float halfTile         = tileWidth/2.0f;
    public static final float oneAndHalfTile   = 3.0f*tileWidth/2.0f;
    public static final float halfField        = X_WIDTH/2.0f;
    public static final float quadField        = X_WIDTH/4.0f;
    public static final float mmTargetHeight   = 6.0f;

    public static final float IMAGE_Z = mmTargetHeight;

    private static final float scale = 25.4f;

    //Note: asset file has 304mm x 224mm (12"x8.8")for RR !?!?
    //Need to figure out what xml coordinates really mean
//    public static int target_width  = 127 * 2;
//    public static int target_height = 92 * 2;
    //It turns out that we really need the trackable page dimensions in this case
    //So use 8.5"x11" paper in landscape =
    public static int target_width  = 280;
    public static int target_height = 216;

    static float[] scaleArr(float[] inArr)
    {
        float[] outArr = {inArr[0], inArr[1], inArr[2]};
        for (int i =0; i<inArr.length; ++i)
        {
            outArr[i]*= Field.scale;
        }
        return outArr;
    }

    public String getAssetName()    { return assetName; }
    public List<String> getImageNames() { return trackableNames; }

    void setImageNames() {}
    void setImageLocations() {}

    @SuppressWarnings("SameParameterValue")
    void setHasVuMarks(boolean hasVuMarks)
    {
        this.hasVuMarks = hasVuMarks;
    }
    public boolean hasVuMarks() {return hasVuMarks;}
    private boolean hasVuMarks = false;

    //public enum ParkPos implements PositionOption
    //{
     //   CENTER_PARK,
       // LEFT_PARK,
        //RIGHT_PARK
    //}

    public enum Route implements PositionOption
    {
        DROP_AND_PARK,
        REPEAT_4PT_JUNCTION

    }

    public enum StartPos implements PositionOption
    {
       START_SAMPLES,
        START_SPECIMENS
   }

    public enum AutonDebug implements PositionOption
    {
        ENABLE,
        DISABLE
    }

    public enum Start_Pos implements PositionOption{
        START_GOAL_RED,
        START_FAR_RED_1AND7,
        START_FAR_BLUE_1AND7,
        START_WALL_BLUE_3,
        START_WALL_RED_3,
        START_WALL_RED4,
        START_WALL_BLUE4,
        START_GOAL_RED5,
        START_GOAL_BLUE5,
        START_FAR_RED6,
        START_FAR_BLUE6,

        START_FAR_RED,
        START_WALL_RED,
        START_GOAL_BLUE,
        START_FAR_BLUE,
        START_WALL_BLUE,

    }

    public enum Highways implements PositionOption
    {
        WALL,
        Park2,
        Park1
    }

    public enum Motif implements PositionOption
    {
        SHOOTGPP,
        SHOOTPGP,
        SHOOTPPG,
    }
    public enum Num_shots implements PositionOption
    {
        THREE,
        FOUR,
    }
    public enum Wiffle_Pos implements PositionOption
    {
        HUMAN1,
        PARK2,
        GATE3,
        GOAL4,
        HUMAN1BLUE,
        PARK2BLUE,
        GATE3BLUE,
        GOAL4BLUE,
    }
    public enum Park_Pos implements PositionOption
    {
        INSIDEPRIMARYPARK1RED,
        INSIDEPRIMARYPARK2RED,
        OUTSIDEPRIMARYPARK1RED,
        OUTSIDEPRIMARYPARK2RED,
        INSIDEPRIMARYPARK1BLUE,
        INSIDEPRIMARYPARK2BLUE,
        OUTSIDEPRIMARYPARK1BLUE,
        OUTSIDEPRIMARYPARK2BLUE,

    }

}
