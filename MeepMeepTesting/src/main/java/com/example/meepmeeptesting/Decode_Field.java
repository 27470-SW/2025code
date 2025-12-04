package com.example.meepmeeptesting;


@SuppressWarnings({"unused", "WeakerAccess"})
public class Decode_Field extends Field
{
    public Decode_Field()
    {
        super("PowerPlay");
        setHasVuMarks(false);
    }

    //Point naming key:
    //1st char: R=Red, B=Blue
    //2nd char: L=Left start, R=Right start (viewed from red side - along field X)
    //3rd-4th chars: Pt description

    private static final String TAG = " SJH_RFD";

    private static final int BLUE  = 0;
    private static final int RED   = 1;
    private static final int STRT1 = 0;
    private static final int STRT2 = 1;
    private static final int LEFT  = 0;
    private static final int CNTR  = 1;
    private static final int RGHT  = 2;


    private static final int ALNC_RED = 0;
    private static final int ALNC_BLU = 1;
    private static final int STRT_ONE = 0;
    private static final int STRT_TWO = 1;
    private static final int STN_LEFT = 0;
    private static final int STN_CNTR = 1;
    private static final int STN_RGHT = 2;

    private static final String ASSET_NAME = "PowerPlay";

    void setImageNames()
    {
       trackableNames.add("BlueStorage");
       trackableNames.add("BlueAllianceWall");
       trackableNames.add("RedStorage");
       trackableNames.add("RedAllianceWall");
    }

    void setImageLocations()
    {
        float[][] TRACKABLE_POS = {
                scaleArr(new float[]{-halfField,  oneAndHalfTile, IMAGE_Z}),
                scaleArr(new float[]{ halfTile,   halfField,      IMAGE_Z}),
                scaleArr(new float[]{-halfField, -oneAndHalfTile, IMAGE_Z}),
                scaleArr(new float[]{ halfTile,  -halfField,      IMAGE_Z})
        };
    }
}
