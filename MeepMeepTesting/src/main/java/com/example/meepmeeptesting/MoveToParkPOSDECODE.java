package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Field.Highways.Park1;
import static com.example.meepmeeptesting.Field.Highways.Park2;
import static com.example.meepmeeptesting.Field.Highways.WALL;
import static com.example.meepmeeptesting.Route.Heading.HEAD_LINEAR;
import static com.example.meepmeeptesting.Route.Movement.LINE;
import static com.example.meepmeeptesting.Route.Movement.SPLINE;
import static com.example.meepmeeptesting.Field.Park_Pos.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MoveToParkPOSDECODE {
    Route route;

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
        }




   /*protected final int sx;
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

