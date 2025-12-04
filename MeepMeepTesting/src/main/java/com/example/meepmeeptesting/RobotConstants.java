package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;


import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused")
public class RobotConstants
{
    public static final double SLW_ANG_ACCEL = Math.toRadians(90);
    //Indicates whether to use InitTrajectories, InitTrajectories2, or Both for the Auton route
  public static TrajEnum trajType = TrajEnum.INIT_TRAJ_2;

  //params map is WIP
  public static Map<String, Double> params = new HashMap<>();

  //ColorSensor on drivetrain
  public static double CL_VALUE_THRESH = 0.45;

  public static double DT_XOFFSET = 0.0;

  //Spinner (duck wheel)
  public static double SP_POWER = 0.0;
  public static double SP_TURBOPWR = 0.8;
  public static double SP_TIMEOUT = 2.5;
  public static double SP_STEP = 0.02;

  //Drop (dumper)
  public static double DP_TIMEOUT = 1.0;
  public static double DP_DUMP_STOW = 0.0;
  public static double DP_DUMP_DEPLOY = 1.0;
  public static double DP_DUMP_DEPLOY_AUTO = 1.0;
  public static double DP_RATE = 1.0;
  public static double DP_PREWAIT = 0.0;
  //Intake
  public static double IN_TIMEOUT = 1.5;
  public static double IN_AUTO_PWR = 1.0;
  public static double IN_AUTO_PWR_OUT = 1.0;
  public static double IN_TELE_PWR = 1.0;
  public static double IN_ARM_EXT_STOW = 0.0;
  public static double IN_ARM_EXT_IN = 0.0;
  public static double IN_ARM_EXT_DEPLOY = 0.2;
  public static double IN_GEAR = 1.0;
  public static double IN_ACT_SPD = 0.0;
  public static double IN_ACT_DLY = 0.5;
  public static double IN_ACT_DUR = 0.5;

  //Elev
  public static int      EL_NUM_LEVS=3;
  public static double   EL_MAX_IPS = 1.0;
  public static double   EL_SPD = 1.0;
  public static double[] EL_LEVS;

  public static double TR_SPD_SCL = 0.5;
  public static double TR_MAX_IPS = 5.0;
  public static double[] TR_LEVS = {0.0, 7.8};

  //Marker
  public static double MK_ARM_STOW   = 0.05;
  public static double MK_ARM_DEPLOY = 0.82;
  public static double MK_ARM_CARRY  = 0.28;

  //Lift
  public static double LF_PRE_TIME = 0.0;
  public static double LF_TIMEOUT = 0.5;

  public static double LF_ARM_ROT_SPD = 0.15;

  public static double LF_ARM_EXT_SPD = 0.25;
  public static double LF_ARM_EXT_PULLEY_DIAM=1.5;
  public static double LF_ARM_EXT_EXT_GR=Math.PI*LF_ARM_EXT_PULLEY_DIAM*2;
  //7252 now uses cascade rigging on 2 stage lif
  public static double[] LF_EXT_LEVS = {0.0, 3.5, 9.5, 15.4}; //0, 142, 386, 627

  public static double LF_STOW_DEG =  0.0;

  public static int LF_ARM_ROT_STOW = 0;
  public static int LF_ARM_ROT_LOW =  0;
  public static int LF_ARM_ROT_MID =  0;
  public static int LF_ARM_ROT_HIGH = 500;
  public static int LF_ARM_ROT_HITL = 500;
  public static int LF_ARM_ROT_GRAB = 500;
  public static int LF_ARM_ROT_MAX =  500;
  public static int LF_ARM_ROT_HERE = Integer.MAX_VALUE;
  public static double LF_ARM_ROT_GEAR = 1.0;

  //Bucketrot
  public static double BK_ROT_STOW = 0.50;
  public static double BK_ROT_LOW  = 0.50;
  public static double BK_ROT_MID  = 0.50;
  public static double BK_ROT_HIGH = 0.50;
  public static double BK_ROT_HITL = 0.50;
  public static double BK_ROT_GRAB = 0.70;
  public static double BK_ROT_XFER = 0.50;

  //BucketArm
  public static double BK_ARM_STOW = 0.20;
  public static double BK_ARM_LOW  = 0.16;
  public static double BK_ARM_MID  = 0.16;
  public static double BK_ARM_HIGH = 0.32;
  public static double BK_ARM_HITL = 0.32;
  public static double BK_ARM_GRAB = 0.92;
  public static double BK_ARM_XFER = 0.50;

  //Loader
  public static double LD_AUTO_PWR = 1.0;
  public static double LD_TELE_PWR = 1.0;

  //ImageProcessing
  public static int    IP_CAM_WID = 640;
  public static int    IP_CAM_HGT = 480;
  public static double IP_IMG_TOP = 0.00;
  public static double IP_IMG_BOT = 1.00;
  public static double IP_IMG_LFT = 0.00;
  public static double IP_IMG_RGT = 1.00;

  //These values are for yellow - override in switch below for marker color
  public static double IP_HUE_MIN = 5.0;
  public static double IP_HUE_MAX = 35.0;
  public static double IP_SAT_MIN = 170.0;
  public static double IP_SAT_MAX = 255.0;
  public static double IP_VAL_MIN = 60.0;
  public static double IP_VAL_MAX = 255.0;

  //DT
  public static final double MMPERIN = 25.4;

  public static double DT_TRACK_WIDTH = 16.34;
  public static double BOT_LEN = 18;
  public static double BOT_WID = 18.0;

  public static Chassis bot= Chassis.B7253;
  public static Field.Alliance alliance = Field.Alliance.RED;


  //CamServo info
  public static double CAM_RED_1 = 0.38;
  public static double CAM_RED_2 = 0.18;
  public static double CAM_BLU_1 = 0.12;
  public static double CAM_BLU_2 = 0.28;
  public static double CAM_STOW  = 0.28;

  //Defender info
  public static double DefendStow = 0.50;
  public static double DefendDeploy1Ring = 0.50;
  public static double DefendDeploy2Ring = 0.50;
  public static double DefendDeploy3Ring = 0.50;


  //Following variables are related to RoadRunner and should be tuned for each bot
  public static double LATERAL_MULTIPLIER = 1.2; //1.12;

  /*
   * These are the feedforward parameters used to model the drive motor behavior. If you are using
   * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
   * motor encoders or have elected not to use them for velocity control, these values should be
   * empirically tuned.
   */

  /*
   * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
   * Set this flag to false if drive encoders are not present and an alternative localization
   * method is in use (e.g., tracking wheels).
   *
   * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
   * from DriveVelocityPIDTuner.
   */
  public static boolean RUN_USING_ENCODER = true;

  /*
   * These values are used to generate the trajectories for you robot. To ensure proper operation,
   * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
   * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
   * small and gradually increase them later after everything is working. All distance units are
   * inches.
   */

  public static double MAX_VEL = 50; //RR tune  maxVel 59.96
  public static double MAX_ACCEL = 44;
  public static double MAX_ANG_VEL = Math.toRadians(180);
  public static double MAX_ANG_ACCEL = Math.toRadians(180);

  public static double SLW_VEL = 35;
  public static double SLW_ACCEL = 30;
  public static double SLW_ANG_VEL = Math.toRadians(180);
  //public static double SLW_ANG_ACCEL = Math.toRadians(120);

  public static TrajectoryVelocityConstraint defVelConstraint =
      new MinVelocityConstraint(Arrays.asList(
          new AngularVelocityConstraint(MAX_ANG_VEL),
          new MecanumVelocityConstraint(MAX_VEL, DT_TRACK_WIDTH)
      ));
  public static TrajectoryAccelerationConstraint defAccelConstraint
      = new ProfileAccelerationConstraint(MAX_ACCEL);

  public static TrajectoryVelocityConstraint slwVelConstraint =
      new MinVelocityConstraint(Arrays.asList(
          new AngularVelocityConstraint(SLW_ANG_VEL),
          new MecanumVelocityConstraint(SLW_VEL, DT_TRACK_WIDTH)
      ));
  public static TrajectoryAccelerationConstraint slwAccelConstraint
      = new ProfileAccelerationConstraint(SLW_ACCEL);

  public static TrajectoryVelocityConstraint stkVelConstraint =
      new MinVelocityConstraint(Arrays.asList(
          new AngularVelocityConstraint(Math.toRadians(90)),
          new MecanumVelocityConstraint(20, DT_TRACK_WIDTH)
      ));

  public static TrajectoryVelocityConstraint telVelConstraint =
      new MinVelocityConstraint(Arrays.asList(
          new AngularVelocityConstraint(Math.toRadians(200)),
          new MecanumVelocityConstraint(56.0, DT_TRACK_WIDTH)
      ));

  public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(7.5, 0, 0);
  public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

  public static final boolean logVerbose = false;

  public static final String TAG = "SJH_RBC";
  private static PositionOption startPos;

  public static int MAX_LOOPS = 1;

  public RobotConstants()
  {
    this(Chassis.B7253);
  }

  public RobotConstants(Chassis chassis)
  {
    init(chassis);
  }

  public static void init(Chassis chas, Field.Alliance alliance, PositionOption startPos)
  {
    RobotConstants.alliance = alliance;
    RobotConstants.startPos = startPos;
    init(chas);
  }

  public static void init(Chassis chas)
  {
    bot = chas;

    Field.StartPos sPos = (Field.StartPos)startPos;

    switch (bot)
    {
      case B7252:
        SP_POWER = 0.40;
        SP_STEP = 0.02;
        SP_TIMEOUT = 2.5;
        SP_TURBOPWR = 1.0;

        IN_AUTO_PWR = 1.0;
        IN_AUTO_PWR_OUT = 0.5;
        IN_TELE_PWR = 1.0;
        IN_TIMEOUT  = 1.5;
        IN_GEAR = 1.0;

        LF_PRE_TIME = 0.0;
        LF_ARM_ROT_STOW = 0;
        LF_ARM_ROT_HIGH = 143;
        LF_ARM_ROT_HITL = 143;
        LF_ARM_ROT_GRAB = 143;
        LF_ARM_ROT_MAX =  143;
        LF_ARM_ROT_HERE = Integer.MAX_VALUE;
        LF_ARM_EXT_SPD = 0.80;
        LF_ARM_ROT_SPD = 0.50;
        LF_ARM_ROT_GEAR = 2.0; //This is variable on 7252 with a linkage/cam

        DP_PREWAIT = 1.0;



        CAM_STOW = 0.11;

        BOT_LEN=18;
        BOT_WID=18;
        MAX_VEL = 50;
        LATERAL_MULTIPLIER = 1.10;
        DT_TRACK_WIDTH = 11.826;
        TRANSLATIONAL_PID = new PIDCoefficients(8.0, 0, 0.0);
        HEADING_PID = new PIDCoefficients(8.0, 0, 0.0);
        RUN_USING_ENCODER = false;

        break;

      case B7253:
        SP_POWER = 0.44;
        SP_TIMEOUT = 2.2;
        SP_TURBOPWR = 1.0;
        SP_STEP = 0.02;

        switch (alliance)
        {
          case RED:
            if (sPos == Field.StartPos.START_SPECIMENS) {
              IP_IMG_TOP = 0.10;
              IP_IMG_BOT = 0.80;
              IP_IMG_LFT = 0.10;
              IP_IMG_RGT = 0.90;
            }
            break;

          case BLUE:
            if (sPos == Field.StartPos.START_SAMPLES) {
              IP_IMG_TOP = 0.20;
              IP_IMG_BOT = 0.80;
              IP_IMG_LFT = 0.22;
              IP_IMG_RGT = 0.78;
            }
            break;
        }

        IP_HUE_MIN = 10.0;
        IP_HUE_MAX = 35.0;
        IP_SAT_MIN = 170.0;
        IP_SAT_MAX = 255.0;
        IP_VAL_MIN = 100.0;
        IP_VAL_MAX = 255.0;

        IN_AUTO_PWR = 0.7;
        IN_TELE_PWR = 0.7;
        IN_TIMEOUT  = 1.0;
        IN_GEAR = 1.0;

        IN_ACT_SPD = -0.25;
        IN_ACT_DLY = 0.25;
        IN_ACT_DUR = 0.5;

        //measured 1 rotation of servo = 120mm=4.72in lift
        //That means effective diam of ~38.2mm=1.5"
        //GoBilda super speed servo 67:1 gearing, noload: 230RPM @6V; 180RPM @4.8V.
        //GoBilda speed servo 135:1 gearing, noload: 115RPM @6V; 90RPM @4.8V.
        //Lift servo is super speed.   CW servo rev w/ High PWM.
        //PWM range for continuous mode may be only 1000-2000
        //HW reports that up is 800 (1000?) and down is 2200 (2000?)
        //So, we should reverse it
        //Assume 6v rev servo power module is being used
        //4.72ipr * 230/60 = 18 ips
        //For initial safety testing, use 6ips
        EL_MAX_IPS = 18.0; //18 ips
        EL_SPD = 1.0;
        EL_NUM_LEVS = 3;
        EL_LEVS = new double[EL_NUM_LEVS];
        EL_LEVS[0] = 0.0;
        EL_LEVS[1] = 6.1;
        EL_LEVS[2] = 12.75;

        DP_TIMEOUT     = 0.5;
        DP_DUMP_STOW   = 0.59;
        DP_DUMP_DEPLOY = 1.0;
        DP_RATE = 2.5;  //(Pct/100) of range per second
        DP_PREWAIT = 1.5;

        CAM_RED_1 = 0.481;
        CAM_RED_2 = 0.320;
        CAM_BLU_1 = 0.222;
        CAM_BLU_2 = 0.325;
        CAM_STOW  = 0.444;

        BOT_LEN = 18;
        BOT_WID = 18;
        MAX_VEL = 50;
        LATERAL_MULTIPLIER = 1.10;
        DT_TRACK_WIDTH = 11.826;
        TRANSLATIONAL_PID = new PIDCoefficients(8.0, 0, 0.0);
        HEADING_PID = new PIDCoefficients(8.0, 0, 0.0);
        RUN_USING_ENCODER = false;
        break;

     default:
        break;
    }

    defVelConstraint =
        new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
            new MecanumVelocityConstraint(MAX_VEL, DT_TRACK_WIDTH)
        ));

    defAccelConstraint
        = new ProfileAccelerationConstraint(MAX_ACCEL);

    slwVelConstraint =
        new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(SLW_ANG_VEL),
            new MecanumVelocityConstraint(SLW_VEL, DT_TRACK_WIDTH)
        ));

    slwAccelConstraint
        = new ProfileAccelerationConstraint(SLW_ACCEL);

    params.put("SP_POWER", SP_POWER);
    params.put("SP_TIMEOUT", SP_TIMEOUT);
    params.put("DP_TIMEOUT", DP_TIMEOUT);
    params.put("IN_TIMEOUT", IN_TIMEOUT);
    params.put("IN_AUTO_PWR", IN_AUTO_PWR);
    params.put("IN_TELE_PWR", IN_TELE_PWR);
    params.put("EL_MAX_IPS", EL_MAX_IPS);
  }

  public enum Chassis
  {
    B7252,
    B7253,
    B7254
  }

  public enum TrajEnum{
    INIT_TRAJ_1,
    INIT_TRAJ_2,
    INIT_TRAJ_BOTH
  }

  public static double getMotorVelocityF(double ticksPerSecond)
  {
    // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
    return 32767 / ticksPerSecond;
  }

  public static void info()
  {
  }
}
