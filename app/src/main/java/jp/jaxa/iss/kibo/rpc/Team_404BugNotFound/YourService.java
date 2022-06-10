package jp.jaxa.iss.kibo.rpc.Team_404BugNotFound;

import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    private final String TAG=this.getClass().getSimpleName();
    @Override
    protected void runPlan1(){
        Log.i (TAG,"Mission start");
        // the mission starts
        api.startMission();

        // move to point1
        Log.i (TAG,"[NOTE] To point1 y=-7.77000");
        moveToLoop(10.71000, -7.77000, 4.48000,0, 0.707f, 0, 0.707f);

        // report point1 arrival
        Log.i (TAG,"Arrive Point 1");
        api.reportPoint1Arrival();

        // get a camera image
        Mat image1 = api.getMatNavCam();
        //save the image
        api.saveMatImage(image1,"image1.png");

        // takeTarget1
        api.laserControl(true);

        api.takeTarget1Snapshot();

        api.laserControl(false);

        //straight path
//        point = new Point(11.27460, -7.7, 4.48);
//        quaternion = new Quaternion(0, 0, -0.707f, 0.707f);
//        api.moveTo(point, quaternion, false);
//        point = new Point(11.27460, -9.92284, 4.48);
//        api.moveTo(point, quaternion, false);
//        point = new Point(11.27460, -9.92284, 5.29881);
//        api.moveTo(point, quaternion, false);

        Log.i (TAG,"[NOTE] To point2");
        //test target2 little calculation
        moveToLoop(11.394, -8.87, 4.48,0, 0, -0.707f, 0.707f);
        //change z=4.62
        moveToLoop(11.394, -9.65, 4.62,0, 0, -0.707f, 0.707f);
        //change from (11.27460-0.07, -9.92284, 5.29881+0.18) to (11.2046, -9.92284, 5.47881)
        moveToLoop(11.2046, -9.92284, 5.47881,0, 0, -0.707f, 0.707f);

        //take a photo
        Mat image2 = api.getMatNavCam();
        api.saveMatImage(image2,"image2.png");
        // takeTarget2
        api.laserControl(true);
        api.takeTarget2Snapshot();
        api.laserControl(false);

        // send mission completion
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    private Result moveToLoop(double PointX, double PointY, double PointZ, float QuaternionX, float QuaternionY, float QuaternionZ, float QuaternionW ){
        Point point = new Point(PointX, PointY, PointZ);
        Quaternion quaternion = new Quaternion(QuaternionX, QuaternionY, QuaternionZ, QuaternionW);
        Result result = api.moveTo(point, quaternion, false);
        final  int LOOP_MAX = 5;
        //check result and loop while moveTo api is not succeeded
        int loopCounter = 0;
        while(!result.hasSucceeded() && loopCounter < LOOP_MAX){
            //retry
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
        }
        Log.i (TAG,"[NOTE] moveTo("+point.getX()+" ,"+point.getY()+" ,"+point.getZ()+") Loop number: "+loopCounter);
        return result;

    }
}

