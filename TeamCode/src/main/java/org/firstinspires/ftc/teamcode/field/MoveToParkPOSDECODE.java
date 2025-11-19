package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Field.Park_Pos.*;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;


public class MoveToParkPOSDECODE {
    String TAG = "PARK_POS_DECODE";
    Route route;
    private Field.Parks stackToBack;
    private Field.Parks pixelStack;

    private Route.TeamElement teamElement;
    private Field.Alliance alliance;
    public MoveToParkPOSDECODE(Route constructorRoute) {
        route = constructorRoute;
    }

    public void makeTraj(Field.Park_Pos parkPos, Field.Alliance alliance) {

        if (parkPos == INSIDEPRIMARYPARK1RED) {
            route.addLocation(route.parkInside1Red, LINE, HEAD_LINEAR);
        } else if (parkPos == INSIDEPRIMARYPARK2RED) {
            route.addLocation(route.parkInside2Red, LINE, HEAD_LINEAR);
        } else if (parkPos == OUTSIDEPRIMARYPARK1RED) {
            route.addLocation(route.parkOutside1Red, LINE, HEAD_LINEAR);
        } else if (parkPos == OUTSIDEPRIMARYPARK2RED) {
            route.addLocation(route.parkOutside2Red, LINE, HEAD_LINEAR);
        } else if (parkPos == INSIDEPRIMARYPARK1BLUE){
            route.addLocation(route.parkInside1Blue, LINE, HEAD_LINEAR);
        } else if (parkPos == INSIDEPRIMARYPARK2BLUE){
            route.addLocation(route.parkInside2Blue, LINE, HEAD_LINEAR);
        } else if (parkPos == OUTSIDEPRIMARYPARK1BLUE){
            route.addLocation(route.parkOutside1Blue, LINE, HEAD_LINEAR);
        }else if (parkPos == OUTSIDEPRIMARYPARK2BLUE) {
            route.addLocation(route.parkOutside2BLUE, LINE, HEAD_LINEAR);

        }
        else if (parkPos == NOPARKRED ){

        }
        else if (parkPos == N0PARKBLUE){

        }
        else {
            if(alliance == Field.Alliance.RED)
                route.addLocation(route.tempParkRed, LINE, HEAD_CONSTANT);
            else
                route.addLocation(route.tempParkBlue, LINE, HEAD_CONSTANT);
        }








   /*rotected final int sx;
    protected final int sy;
    protected int sh;
    protected int sf;
    protected int sr;
    protected final double flip;
    protected final double strtX;
    protected final double strtY;
    protected final double strtH;
    protected Field.Alliance alliance;
    protected PositionOption startPos;
    protected final double botBackToCtr;
    protected final double botSideToCtr;
    double rightSideLineUpToBorderAdjustment = 0.5;

    protected Pose2d moveFromStart;
    protected Pose2d start;*/

    }
}

