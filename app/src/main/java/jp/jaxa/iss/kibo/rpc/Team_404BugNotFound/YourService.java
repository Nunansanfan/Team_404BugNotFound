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

        // move to a point
        Point point = new Point(10.71000, -7.77000, 4.48000);
        Quaternion quaternion = new Quaternion(0, 0.707f, 0, 0.707f);
        Log.i (TAG,"to point 1 y=-7.77000");

        Result result = api.moveTo(point, quaternion, false);
        final  int LOOP_MAX = 5;
        //check result and loop while moveTo api is not succeeded
        int loopCounter = 0;
        while(!result.hasSucceeded() && loopCounter < LOOP_MAX){
            //retry
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
            Log.i (TAG,"Loop number"+loopCounter);
        }

        // report point1 arrival
        Log.i (TAG,"Arrive Point 1");
        api.reportPoint1Arrival();

        // get a camera image
        Mat image = api.getMatNavCam();


        // irradiate the laser
        api.laserControl(true);

        //save the image
        api.saveMatImage(image,"image1.png");

        api.takeTarget1Snapshot();

        // turn the laser off
        api.laserControl(false);
        /* ******************************************** */
        /* write your own code and repair the air leak! */
        /* ******************************************** */

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

}

