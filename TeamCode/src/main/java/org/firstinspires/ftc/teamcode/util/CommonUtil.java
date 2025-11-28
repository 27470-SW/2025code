package org.firstinspires.ftc.teamcode.util;

import android.app.Activity;
import android.content.Context;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.PositionOption;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings({"unused", "FieldCanBeLocal"})
public class CommonUtil
{
    private static Field.Alliance alliance;
    private static PositionOption startPos;
    private static final String TAG = "SJH_COM";

    private static final int topLayoutViewId = R.id.entire_screen;
    private static final int cameraViewId = R.id.cameraMonitorViewId;
    public static List<Integer> viewIds = new ArrayList<>(2);

    static
    {
        RobotLog.dd(TAG, "Init static block");
    }

    private static HardwareMap  h;
    private static OpMode       o;
    private static LinearOpMode l;
    private static Telemetry    t;
    private static HalDashboard d;
    private static DataLogger   dl;
    private static TelemetryPacket tp;
    private static FtcDashboard dbd;

    private final boolean logData   = true;

    public boolean hasCam = false;
    private boolean useOpenCV  = false;

    private boolean useVuforia = false;
    private final boolean vuforiaUseScreen = true;


    public static OpenCvCamera camera;
    public static boolean cameraOpened = false;

    private CommonUtil()
    {
    }

    private static class SingletonHelper
    {
        private static final CommonUtil instance = new CommonUtil();
    }

    public static CommonUtil getInstance()
    {
        return SingletonHelper.instance;
    }

    public void init(OpMode o, boolean useOcv, Field.Alliance alliance, PositionOption side)
    {
        this.useOpenCV  = useOcv;

        CommonUtil.o = o;
        initOpModeProps();
        initDataLogger();
        initDashboard();
        initContextAct();

        if(!useOcv)
        {
            viewIds.add(0, cameraViewId);
        }
        else
        {
            int[] viewportContainerIds =
              OpenCvCameraFactory
                .getInstance()
                .splitLayoutForMultipleViewports
                  (cameraViewId, 2,
                   OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
            viewIds.add(0, viewportContainerIds[0]);
            viewIds.add(1, viewportContainerIds[1]);
        }

        //initOpenCV(alliance, side);
    }

    private  void initOpModeProps()
    {
        CommonUtil.h = o.hardwareMap;
        CommonUtil.t = o.telemetry;
        if (o instanceof LinearOpMode)
            CommonUtil.l = (LinearOpMode) o;
    }

    private void initDataLogger()
    {
        if(dl != null) dl.closeDataLogger();
        if (logData)
        {
            dl = new DataLogger();
        }
    }

    private void initDashboard()
    {
        CommonUtil.d = HalDashboard.createInstance(t);
        CommonUtil.dbd = FtcDashboard.getInstance();
        CommonUtil.dbd.setTelemetryTransmissionInterval(25);
    }

    private void initContextAct()
    {
    }

    private void initOpenCV(Field.Alliance alliance, PositionOption side)
    {
        RobotLog.dd(TAG, "initOpenCV: " +
                " useOpenCV: " + useOpenCV);
        if(!useOpenCV) return;

        RobotLog.dd(TAG, "Initializing OpenCV/camera");

        WebcamName webcamName;
        if(
                (alliance == Field.Alliance.RED && side == Field.StartPos.START_SAMPLES) ||
                        (alliance == Field.Alliance.BLUE && side == Field.StartPos.START_SAMPLES)
        )
        {
            webcamName = h.get(WebcamName.class, "webcamRIGHT");
        }
        else
        {
            webcamName = h.get(WebcamName.class, "webcamRIGHT");
        }

        if(webcamName != null)
        {
            RobotLog.dd(TAG, "Initializing" + webcamName);
            hasCam = true;
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(webcamName, viewIds.get(0));

            // Set the viewport renderer here before opening camera
            camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);
        }

        if(camera == null)
        {
            RobotLog.ee(TAG, "Problem Creating camera");
            return;
        }

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cameraOpened = true;
                RobotLog.dd(TAG, "Camera Opened");
                //removed view port render call
                camera.startStreaming(RobotConstants.IP_CAM_WID, RobotConstants.IP_CAM_HGT, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {
                RobotLog.ee(TAG, "Error with camera: %d", errorCode);
            }
        });


        RobotLog.dd(TAG, "Back from Initializing OpenCV");
    }

    public HardwareMap getHardwareMap()
    {
        return h;
    }

    public OpMode getOpMode()
    {
        return o;
    }

    public LinearOpMode getLinearOpMode()
    {
        return l;
    }

    public Telemetry getTelemetry()
    {
        return t;
    }

    public HalDashboard getDashboard()
    {
        return d;
    }

    public FtcDashboard getFtcDashboard()
    {
        return dbd;
    }

    public TelemetryPacket getTelemetryPacket()
    {
        return tp;
    }

    public void setTelemetryPacket(TelemetryPacket telemPacket)
    {
        tp = telemPacket;
    }

    public DataLogger getDataLogger()
    {
        return dl;
    }

    public Activity getActivity()
    {
        Context context = h.appContext;
        Activity act = null;
        if(context instanceof Activity)
        {
            act = (Activity) context;
        }
        return act;
    }

    @SuppressWarnings("WeakerAccess")
    public Context getContext()
    {
        return h.appContext;
    }

    public FtcRobotControllerActivity getApp()
    {
        return (FtcRobotControllerActivity)getActivity();
    }

    public int getCameraMonitorViewId() { return cameraViewId; }

    private View getMainView()
    {
        final Activity act = getActivity();
        return act.getCurrentFocus().getRootView();
    }
}
