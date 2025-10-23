package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import static com.example.meepmeeptesting.Field.StartPos.*;
import static com.example.meepmeeptesting.DECODE_Route.preLoadedCone.high;
import static com.example.meepmeeptesting.DECODE_Route.preLoadedCone.low;
import static com.example.meepmeeptesting.DECODE_Route.preLoadedCone.lowHigh;
import static com.example.meepmeeptesting.DECODE_Route.preLoadedCone.med;
import static com.example.meepmeeptesting.Field.Start_Pos.*;
import static com.example.meepmeeptesting.Route.Movement.*;
import static com.example.meepmeeptesting.Route.Heading.*;

public class DECODE_Route extends Route
{
  public DECODE_Route(
                          PositionOption startPos,
						  Field.Park_Pos parkPos,
                    Field.Wiffle_Pos firstLocation)
  {
    super(startPos, parkPos, firstLocation);
  }

 public enum preLoadedCone{low,med,high,lowHigh}
    public static preLoadedCone preLoadedConeJunc = low;
    public static preLoadedCone firstConeStackCone = med;
    public static preLoadedCone secondConeStackCone = lowHigh;
    public static preLoadedCone thirdConeStackCone = high;


    public void firstConeStackConeJunc()
    {
        if (firstConeStackCone == low)
        {


        }

        if (firstConeStackCone == med)
        {

        }

        if (firstConeStackCone == high)
        {

        }

        if (firstConeStackCone == lowHigh)
        {

        }
    }


    public void secondConeStackConeJunc(){
    if (secondConeStackCone == low)
    {

    }

    if (secondConeStackCone == med)
    {

    }

    if (secondConeStackCone == high)
    {

    }
        if (secondConeStackCone == lowHigh)
        {

        }
}


    public void thirdConeStackConeJunc(){
    if (thirdConeStackCone == low)
    {

    }

    if (thirdConeStackCone == med)
    {

    }

    if (thirdConeStackCone == high)
    {

    }
        if (thirdConeStackCone == lowHigh)
        {

        }
}
  protected void moveFromStart()
  {

       addLocation(start,LINE, HEAD_LINEAR);
      //addLocation(extra1, LINE, HEAD_LINEAR);
      //addLocation(Start, LINE, HEAD_LINEAR);
      //addLocation(awayFromStart, LINE, HEAD_LINEAR);
      /*addFunction( this::liftElvLowPoleJnct);
      addLocation(extra3, LINE, HEAD_LINEAR);
      addLocation(extra2, LINE, HEAD_SPLINE);
      addFunction( this::elvToConeStack);
      addEvent(Action.WAIT, 0.5);
      addFunction( this::openClaw);
      addLocation(extra3, LINE, HEAD_SPLINE);
      addLocation(preGrabCone[0], LINE, HEAD_SPLINE);
      addLocation(strafeGrabCone[0], LINE, HEAD_LINEAR);
      addFunction( this::closeClaw);
      addEvent(Action.WAIT, 0.3);*/
      //addFunction( this::liftElvMediumPoleJnct);
      //addLocation(toTurnBox, LINE, HEAD_LINEAR);
  }

    protected void randomOne()
    {
        firstConeStackConeJunc();
        secondConeStackConeJunc();
        thirdConeStackConeJunc();
    }



  protected void goPark()
  {

  }

  /*
  Use the builder functions in Route to build routes in initTrajectories2
  common examples are:
        addLocation(Pose2d, Movement, Heading) - to move the robot to that location
        addFunction(functionName) - to perform a function
        addEvent(WAIT, waitTime) - to add a wait period
        addParkPosition(ParkPos, Pose2d, Movement, Heading) - to define where to park and how to move there
  */

    public void placeTapePixel(){
        System.out.println("{Place Pixel on Spike Tape}");
    }

    public void dropPixel(){
        //TODO: add code to make this work
        System.out.println("Drop Pixel\n");
    }

    public void dropPixels(){
        dropPixel();
        dropPixel();
    }

    public void pickUpPixel(){
        System.out.println("Grab pixel from stack");
    }

    public void pickUpPixels(){
        pickUpPixel();
        pickUpPixel();
    }


   protected void initTrajectories2()
   {

       Pose2d lastPose;
       if(startPos == START_SAMPLES) {
           SampleRoute t1 = new SampleRoute(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }
       else if(startPos == START_SPECIMENS)
       {
           SpecimenRoute t1 = new SpecimenRoute(this);
           t1.makeTraj(startPos, parkPos, lastLocation);

       }
       else if( startPos == START_FAR_RED_1AND7)
       {
           F1F3N_Route1and7Red t1 = new F1F3N_Route1and7Red(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }

       else if( startPos == START_FAR_BLUE_1AND7)
       {
           F1F3N_Route1and7Red t1 = new F1F3N_Route1and7Red(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }

       MoveToParkPOSDECODE t1 = new MoveToParkPOSDECODE(this);
       t1.makeTraj(parkPos,alliance);

       lastPose = this.getEnd();

        //Always do this at the end of initTrajectories2
        finalizeTrajSeq();
    }

    protected void statesCompetitionRoute()
    {}


    protected void pointToPointTest() 
	{
        System.out.println("Test Pt to Pt routes");


        /* Instantiate builder - the builder has start point */
//        TrajectorySequenceBuilder start = new TrajectorySequenceBuilder(
//                oppositionHighJunctionDrop,
//                defVelLim, defAccelLim,
//                RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);
//
//        /* Define/describe the trajectory */
//        start
//                .setTangent(reverseOppositionhighJunctionDropSplineTangent)
//                .splineToSplineHeading(coneStackGrabAJUSTED_YX, coneStackGrabAJUSTED_YX.getHeading())
//                .addTemporalMarker(this::closeClaw)
//                .waitSeconds(0.25)
//                .addTemporalMarker(this::liftElvLowPoleJnct)
//                .waitSeconds(0.1)
//                //.turn(sx*sy*Math.toRadians(90)+flip)
//                .setTangent(thelowtanjent)
//                .splineToLinearHeading(lowJuncStraightOn, sx * sy * Math.toRadians(160) + flip)
//                .addTemporalMarker(this::elvToConeStack)
//                .waitSeconds(0.01)
//                .addTemporalMarker(this::openClaw)
//                //.setTangent(thelowtanjentreverse)
//                .lineToLinearHeading(new Pose2d(sx * 60, sy * 30, sx *sy * Math.toRadians(270) + flip));
//        /* Full does not need to have a trajectory sequence */
//        start.build();
//        /////////////////////////////////////////////////////////////////////
//
//
//        //segmentStateMap.put(SegmentState.LOW_JUNCTION_CONE_DROP, tsq_lowJunctionConeDrop);
//
//
//        double totalDur = 0.0;
//        /* Aggregates duration for all the route sequences
//         *  Total Time needs to be less than 30 secs
//         */
//        for (Map.Entry<SegmentState, TrajectorySequence> e : segmentStateMap.entrySet()) {
//            String seqName = e.getKey().name();
//            TrajectorySequence tSeq = e.getValue();
//            double dur = tSeq.duration();
//            System.out.printf(Locale.US, "Sequence %s dur: %.2f start: %.2f%n",
//                    seqName, dur, totalDur);
//            totalDur += dur;
//        }
//        System.out.printf(Locale.US, "Total duration: %.2f%n", totalDur);
//        System.out.println("Done Building trajectories");
//
//        double cumTime = 0.0;
//        System.out.printf(Locale.US, "BOT:%s ALLIANCE:%s START:%s PARK:%s%n",
//                RobotConstants.bot, alliance, startPos, parkPos);
//
//        for (Map.Entry<SegmentState, TrajectorySequence> e : segmentStateMap.entrySet()) {
//            String seqName = e.getKey().name();
//            TrajectorySequence tSeq = e.getValue();
//            System.out.printf(Locale.US, "Trajectory Sequence %s len=%d dur=%.2f%n",
//                    seqName, tSeq.size(), tSeq.duration());
//            for (int i = 0; i < tSeq.size(); i++) {
//                SequenceSegment seg = tSeq.get(i);
//
//                if (seg instanceof TrajectorySegment) {
//                    /* Trajectory sequences are added to the full Trajectory Sequence from the route state map
//                     *  in the order the Enumeration of the EnumMap
//                     *  Full is the default one that should have the pStart
//                     */
//                    Trajectory t = ((TrajectorySegment) seg).getTrajectory();
//                    start.addTrajectory(t);
//                    System.out.printf(Locale.US, "  TrajSeg %d %s to %s in %.2f at %.2f%n",
//                            i, seg.getStartPose(), seg.getEndPose(), seg.getDuration(), cumTime);
//                    for (TrajectoryMarker m : ((TrajectorySegment) seg).getTrajectory().getMarkers()) {
//                        System.out.printf(Locale.US, "    marker in Traj @ %.2f%n", m.getTime());
//                    }
//                    cumTime += t.duration();
//                } else {
//                    if (seg instanceof TurnSegment) {
//                        TurnSegment ts = (TurnSegment) seg;
//                        start.turn(ts.getTotalRotation());
//                        System.out.printf(Locale.US, "  TurnSeg %d %s to %s in %.2f at %.2f%n",
//                                i, seg.getStartPose(), seg.getEndPose(), seg.getDuration(), cumTime);
//                        for (TrajectoryMarker m : seg.getMarkers()) {
//                            start.addTemporalMarker(m.getTime(), m.getCallback());
//                            System.out.printf(Locale.US, "    marker in Turn @ %.2f%n", m.getTime());
//                        }
//                        cumTime += ts.getDuration();
//                    } else if (seg instanceof WaitSegment) {
//                        WaitSegment ws = (WaitSegment) seg;
//                        start.waitSeconds(ws.getDuration());
//                        System.out.printf(Locale.US, "  WaitSeg %d %s to %s in %.2f at %.2f%n",
//                                i, seg.getStartPose(), seg.getEndPose(), seg.getDuration(), cumTime);
//                        for (TrajectoryMarker m : seg.getMarkers()) {
//                            start.addTemporalMarker(m.getTime(), m.getCallback());
//                            System.out.printf(Locale.US, "    marker in Wait @ %.2f%n", m.getTime());
//                        }
//                        cumTime += ws.getDuration();
//                    }
//                }
//            }
//        }
//        /* Store the full route map after it's done being built */
//        fullSeq = start.build();
    }
}
