package jp.jaxa.iss.kibo.rpc.Team_404BugNotFound;

import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import java.util.ArrayList;
import java.util.List;
import java.util.Collections;
import org.opencv.aruco.*;
import org.opencv.core.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgproc.*;
import org.opencv.core.Size;
import org.opencv.core.Rect;

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
        moveToLoop(11.2026, -9.92284, 5.46881,0, 0, -0.707f, 0.707f);

        //init variable value
        int dictID = Aruco.DICT_5X5_250;
        double fx = 523.105750;
        double cx = 635.434258;
        double fy = 534.765913;
        double cy = 500.335102;
        double[] camMatrix = new double[] { fx, 0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 };
        double[] distortionArray = new double[] {-0.164787, 0.020375, -0.001572, -0.000369, 0.000000};
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
        cameraMatrix.put(0, 0, camMatrix);// (row, col,int[])
        Mat dstMatrix = new Mat(1, 5, CvType.CV_64FC1);
        dstMatrix.put(0, 0, distortionArray);
        MatOfDouble distortion = new MatOfDouble();
        distortion.fromArray(distortionArray);
        List<Mat> objP2 = new ArrayList<Mat>();
        MatOfInt board2ID = new MatOfInt();
        setBoard(objP2,board2ID);

        ArrayList<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        Dictionary dict = Aruco.getPredefinedDictionary(dictID);
        Board t2_board = Board.create(objP2, dict, board2ID);

        //take a photo
        Mat image2 = api.getMatNavCam();
        api.saveMatImage(image2,"target2.png");
        Mat processImage = new Mat(image2.rows(), image2.cols(), image2.type());
        image2.copyTo(processImage);
        Mat originalImg = new Mat(image2.rows(), image2.cols(), image2.type());
        image2.copyTo(originalImg);
        Aruco.detectMarkers(processImage, dict, corners, ids);
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        Aruco.estimatePoseBoard(corners, ids, t2_board, cameraMatrix, dstMatrix, rvec, tvec);


        List<MatOfPoint3f> offset_c = new ArrayList<MatOfPoint3f>();
        find_ROI3D(rvec, tvec, offset_c);
        List<MatOfPoint3f> offset = offset_c;

        Mat rMat=new Mat();
        Calib3d.Rodrigues(rvec,rMat);

        Log.i (TAG,"[NOTE] check rvec[0] = "+rMat.get(0,0)[0]+", "+rMat.get(0,1)[0]+", "+rMat.get(0,2)[0]);
        Log.i (TAG,"[NOTE] check rvec[1] = "+rMat.get(1,0)[0]+", "+rMat.get(1,1)[0]+", "+rMat.get(1,2)[0]);
        Log.i (TAG,"[NOTE] check rvec[2] = "+rMat.get(2,0)[0]+", "+rMat.get(2,1)[0]+", "+rMat.get(2,2)[0]);


        double tarx = (double) tvec.get(0, 0)[0];
        double tary = (double) tvec.get(1, 0)[0];
        double tarz = (double) tvec.get(2, 0)[0];
        Point3 tar_pos = new Point3(tarx, tary, tarz);
        Log.i (TAG,"[NOTE] check tvec = "+tarx+", "+tary+", "+tarz);

        double[][] rtmatrix={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
//        double[][] rtmatrix={{rMat.get(0,0)[0],rMat.get(0,1)[0],rMat.get(0,2)[0],tarx}
//                ,{rMat.get(1,0)[0],rMat.get(1,1)[0],rMat.get(1,2)[0],tary}
//                ,{rMat.get(2,0)[0],rMat.get(2,1)[0],rMat.get(2,2)[0],tarz}
//                ,{0,0,0,1}};
        double[][] camMatrixForMult = new double[][] { {fx, 0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0} };
        double[][] iMatrix={{1,0,0,0},{0,1,0,0},{0,0,1,0}};


        MatOfPoint3f _target3D = new MatOfPoint3f();
        _target3D.fromArray(tar_pos);
        MatOfPoint2f _targetImagePlane = new MatOfPoint2f();
        Mat _rvec = new Mat(1, 3, CvType.CV_64FC1);
        Mat _tvec = new Mat(1, 3, CvType.CV_64FC1);

        double[] _r = new double[] { 0.0f, 0.0f, 0.0f };
        double[] _t = new double[] { 0.0f, 0.0f, 0.0f };
        _rvec.put(0, 0, _r);
        _tvec.put(0, 0, _t);

        // find center of marker in 2D image
        Calib3d.projectPoints(_target3D, _rvec, _tvec, cameraMatrix, distortion, _targetImagePlane);

        int cpx = (int) _targetImagePlane.get(0, 0)[0];
        int cpy = (int) _targetImagePlane.get(0, 0)[1];
//        Point center = new Point(cpx,cpy);
//		Imgproc.circle(imgProc.processedImg, center, 4, new Scalar(0,255,255), -1);
        Calib3d.drawFrameAxes(processImage, cameraMatrix, dstMatrix, rvec, rvec, (float) 0.05);

        Log.i (TAG,"[NOTE] cpx = "+cpx+" cpy = "+cpy);

        List<org.opencv.core.Point> ROI_points = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            Calib3d.projectPoints(offset.get(i), _rvec, _tvec, cameraMatrix, distortion, _targetImagePlane);
            int _cpx = (int) _targetImagePlane.get(0, 0)[0];
            int _cpy = (int) _targetImagePlane.get(0, 0)[1];
            org.opencv.core.Point _center = new org.opencv.core.Point(_cpx, _cpy);
            ROI_points.add(_center);
        }
        // find Region of interest
        Rect target_rect = new Rect();
        Mat[] retArr = find_paper(processImage, ROI_points, originalImg,target_rect);
        Mat warped_img = retArr[0];
        Mat cropped_img = retArr[1];
        // now we got warped_img , and cropped_img
        Log.i (TAG,"[NOTE] cropped image cols : "+cropped_img.cols()+", rows : "+cropped_img.rows());

        // find contour
        // can't use ????
        //Imgproc.rectangle(processImage, target_rect, new Scalar(0, 0, 255));

        // set cropped image back to original image
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchey = new Mat();
//        Mat cropped_gray = new Mat(cropped_img.rows(),cropped_img.cols(),cropped_img.type());
//
//        Imgproc.cvtColor(cropped_img, cropped_gray, Imgproc.COLOR_BGR2GRAY);
        Mat binaryImg = new Mat();
        Imgproc.threshold(cropped_img, binaryImg, 100, 200, Imgproc.THRESH_BINARY_INV);

        int x=0;
        int y=0;
        Imgproc.findContours(binaryImg, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = new Scalar(0, 255.0, 0);
            // Drawing Contours
            if (hierarchey.get(0, i)[2] == -1.0) {
                MatOfPoint2f ct2f = new MatOfPoint2f(contours.get(i).toArray());
                Moments moment = Imgproc.moments(ct2f);

                x = (int) (moment.get_m10() / moment.get_m00());
                y = (int) (moment.get_m01() / moment.get_m00());

                Imgproc.circle(cropped_img, new org.opencv.core.Point(x, y), 2, new Scalar(0, 0, 255), -1);
                Log.i (TAG,"[NOTE] Centroid x = "+x+" y = "+y);

            }
            Imgproc.drawContours(cropped_img, contours, i, color, 1, Imgproc.LINE_8, hierarchey, 1,
                    new org.opencv.core.Point());
        }
        cropped_img.copyTo(processImage.submat(target_rect));
        api.saveMatImage(cropped_img,"cropped_img.png");
        api.saveMatImage(processImage,"proc_img.png");

        double height=13.3;
        double width=17.5;
        int croppedRows = cropped_img.rows();
        int croppedCols = cropped_img.cols();
        double cmPerPixRow=height/croppedRows;
        double cmPerPixCols=width/croppedCols;

        double centerX=croppedCols/2;
        double centerY=croppedRows/2;

        double moveXcoor=x-centerX;
        double moveYcoor=y-centerY;

        double xCoorFullPic=cpx+moveXcoor;
        double yCoorFullPic=cpy+moveYcoor;

        double moveXmetre=(moveXcoor*cmPerPixCols)/100;
        double moveYmetre=(moveYcoor*cmPerPixRow)/100;

        Log.i (TAG,"[NOTE] moveXmetre = "+moveXmetre+", moveYmetre = "+moveYmetre);
        Log.i (TAG,"[NOTE] Full Pic x = "+xCoorFullPic+", y = "+yCoorFullPic);

        // x - , z +
//        relativeMoveToLoop(tarx-0.0572,-0.27284,tary+0.1111,0,0,-0.707f,0.707f);
        // this is close
//        relativeMoveToLoop(tarx-0.0994+,0,tary+0.0285+moveYmetre,0,0,-0.707f,0.707f);
        // x-0.0994 , y+0.0285 is the correct offset
        // usable------------------------------------------------------
//        relativeMoveToLoop(tarx+moveXmetre-0.0994,0,tary+moveYmetre+0.0285,0,0,-0.707f,0.707f);
        //-------------------------------------------------------------
//        relativeMoveToLoop(tarx-0.0994+moveXmetre,-0.27284,tary+0.0285-moveYmetre,0,0,-0.707f,0.707f);
//        moveToLoop(11.2026, -9.92284, 5.46881,0, 0, -0.707f, 0.707f);

        double[] centroidCoordinate={xCoorFullPic,yCoorFullPic,1};

        //Cramer's Law
        double[][] matrixA = multiplyMatrices(camMatrixForMult,iMatrix);
        //removable
//        matrixA = multiplyMatrices(matrixA,rtmatrix);

        double a03=matrixA[0][3];
        matrixA[0][3]=0;
        double a13=matrixA[1][3];
        matrixA[1][3]=0;
        double a23=matrixA[2][3];
        matrixA[2][3]=0;
        double detA=determinantOfMatrix(matrixA,3);

        double realX=0, realY=0, realZ=0;
        if(detA!=0) {
            //x
            double[][] tempxMat = {{centroidCoordinate[0]-a03, matrixA[0][1], matrixA[0][2]}
                    , {centroidCoordinate[1]-a13, matrixA[1][1], matrixA[1][2]}
                    , {centroidCoordinate[2]-a23, matrixA[2][1], matrixA[2][2]}};
            double detTempXMat = determinantOfMatrix(tempxMat, 3);
            realX=detTempXMat/detA;

            //y
            double[][] tempyMat = {{matrixA[0][0],centroidCoordinate[0]-a03, matrixA[0][2]}
                    , {matrixA[1][0], centroidCoordinate[1]-a13, matrixA[1][2]}
                    , {matrixA[2][0], centroidCoordinate[2]-a23, matrixA[2][2]}};
            double detTempYMat = determinantOfMatrix(tempyMat, 3);
            realY=detTempYMat/detA;

            //z
            double[][] tempzMat = {{matrixA[0][0],matrixA[0][1], centroidCoordinate[0]-a03}
                    , {matrixA[1][0],matrixA[1][1], centroidCoordinate[1]-a13}
                    , {matrixA[2][0],matrixA[2][1], centroidCoordinate[2]-a23}};
            double detTempZMat = determinantOfMatrix(tempzMat, 3);
            realZ=detTempZMat/detA;
        }
        else{
            Log.i (TAG,"[NOTE] error det = 0");
        }

        Log.i (TAG,"[NOTE] realX,realY,realZ = "+realX+", "+realY+", "+realZ);
//        relativeMoveToLoop(-0.0994+realX,0,+0.0285+realY,0,0,-0.707f,0.707f);
        relativeMoveToLoop(-0.0572+realX,0,+0.1111+realY,0,0,-0.707f,0.707f);


        // takeTarget2
        api.laserControl(true);
        api.takeTarget2Snapshot();
        api.laserControl(false);

        Log.i (TAG,"[NOTE] To goal");
        moveToLoop(11.27460, -9.65, 4.62063,0, 0, -0.707f, 0.707f);
        moveToLoop(11.27460, -8.3, 4.62063,0, 0, -0.707f, 0.707f);
        moveToLoop(11.27460, -7.89178, 4.96538,0, 0, -0.707f, 0.707f);

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


    public static double[][] matrixTranspose(double[][] m){
        double[][] ret=new double[m[0].length][m.length];
        for(int i=0;i<m[0].length;i++){
            for(int j=0;j<m.length;j++){
                ret[i][j]=m[j][i];
            }
        }
        return  ret;
    }

    private static double[][] MatrixMultiply(double[][] A ,double[][] B){
        double[][] Mul = new double[A.length][B[0].length];
        if(A[0].length!=B.length)
            return null;
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < B[0].length; j++) {
                for (int k = 0; k < B.length; k++)
                    Mul[i][j] += A[i][k] * B[k][j];
            }
        }
        return Mul;
    }
    // Function to get cofactor of
    // mat[p][q] in temp[][]. n is
    // current dimension of mat[][]
    static void getCofactor(double mat[][], double temp[][],
                            int p, int q, int n)
    {
        int i = 0, j = 0;

        // Looping for each element
        // of the matrix
        for (int row = 0; row < n; row++) {
            for (int col = 0; col < n; col++) {
                // Copying into temporary matrix
                // only those element which are
                // not in given row and column
                if (row != p && col != q) {
                    temp[i][j++] = mat[row][col];
                    // Row is filled, so increase
                    // row index and reset col index
                    if (j == n - 1) {
                        j = 0;
                        i++;
                    }
                }
            }
        }
    }

    /* Recursive function for finding determinant
    of matrix. n is current dimension of mat[][]. */
    static double determinantOfMatrix(double mat[][], int n)
    {
        double D = 0; // Initialize result

        // Base case : if matrix
        // contains single element
        if (n == 1)
            return mat[0][0];

        // To store cofactors
        double temp[][] = new double[n][n];

        // To store sign multiplier
        int sign = 1;

        // Iterate for each element of first row
        for (int f = 0; f < n; f++) {
            // Getting Cofactor of mat[0][f]
            getCofactor(mat, temp, 0, f, n);
            D += sign * mat[0][f]
                    * determinantOfMatrix(temp, n - 1);

            // terms are to be added
            // with alternate sign
            sign = -sign;
        }

        return D;
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

    private Result relativeMoveToLoop(double PointX, double PointY, double PointZ, float QuaternionX, float QuaternionY, float QuaternionZ, float QuaternionW ){
        Point point = new Point(PointX, PointY, PointZ);
        Quaternion quaternion = new Quaternion(QuaternionX, QuaternionY, QuaternionZ, QuaternionW);
        Result result = api.relativeMoveTo(point, quaternion, false);
        final  int LOOP_MAX = 5;
        //check result and loop while moveTo api is not succeeded
        int loopCounter = 0;
        while(!result.hasSucceeded() && loopCounter < LOOP_MAX){
            //retry
            result = api.relativeMoveTo(point, quaternion, false);
            ++loopCounter;
        }
        Log.i (TAG,"[NOTE] relativeMoveTo("+point.getX()+" ,"+point.getY()+" ,"+point.getZ()+") Loop number: "+loopCounter);
        return result;

    }

    private static void setBoard(List<Mat> objP, MatOfInt boardID){

        int[] id = new int[] { 11, 12, 13, 14 };
        boardID.fromArray(id);

        List<Point3> c0 = new ArrayList<>();
        List<Point3> c1 = new ArrayList<>();
        List<Point3> c2 = new ArrayList<>();
        List<Point3> c3 = new ArrayList<>();

        c0.add(new Point3(0.0875f, 0.0665f, 0.0f));
        c0.add(new Point3(0.1375f, 0.0665f, 0.0f));
        c0.add(new Point3(0.1375f, 0.0165f, 0.0f));
        c0.add(new Point3(0.0875f, 0.0165f, 0.0f));

        c1.add(new Point3(-0.1375f, 0.0665f, 0.0f));
        c1.add(new Point3(-0.0875f, 0.0665f, 0.0f));
        c1.add(new Point3(-0.0875f, 0.0165f, 0.0f));
        c1.add(new Point3(-0.1375f, 0.0165f, 0.0f));

        c2.add(new Point3(-0.1375f, -0.0165f, 0.0f));
        c2.add(new Point3(-0.0875f, -0.0165f, 0.0f));
        c2.add(new Point3(-0.0875f, -0.0665f, 0.0f));
        c2.add(new Point3(-0.1375f, -0.0665f, 0.0f));

        c3.add(new Point3(0.0875f, -0.0165f, 0.0f));
        c3.add(new Point3(0.1375f, -0.0165f, 0.0f));
        c3.add(new Point3(0.1375f, -0.0665f, 0.0f));
        c3.add(new Point3(0.0875f, -0.0665f, 0.0f));

        MatOfPoint3f c_id0 = new MatOfPoint3f();
        MatOfPoint3f c_id1 = new MatOfPoint3f();
        MatOfPoint3f c_id2 = new MatOfPoint3f();
        MatOfPoint3f c_id3 = new MatOfPoint3f();

        c_id0.fromList(c0);
        c_id1.fromList(c1);
        c_id2.fromList(c2);
        c_id3.fromList(c3);

        objP.add(c_id0);
        objP.add(c_id1);
        objP.add(c_id2);
        objP.add(c_id3);
    }

    public static void find_ROI3D(Mat rvec, Mat tvec, List<MatOfPoint3f> offset_c) {
        List<Point3> global_corner = new ArrayList<>();
        Mat rot = new Mat();
        Calib3d.Rodrigues(rvec, rot);
        double[][] offset_corner = { new double[] { 0.075f, -0.075f, -0.075f, 0.075f },
                new double[] { 0.0625f, 0.0625f, -0.0625f, -0.0625f }, new double[] { 0f, 0f, 0f, 0f }, };

        double[][] rotationMatrix = { new double[] { rot.get(0, 0)[0], rot.get(0, 1)[0], rot.get(0, 2)[0] },
                new double[] { rot.get(1, 0)[0], rot.get(1, 1)[0], rot.get(1, 2)[0] },
                new double[] { rot.get(2, 0)[0], rot.get(2, 1)[0], rot.get(2, 2)[0] } };

        double[][] global_offset = multiplyMatrices(rotationMatrix, offset_corner);
        Point3 tar_c0 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][0],
                (double) tvec.get(1, 0)[0] + global_offset[1][0], (double) tvec.get(2, 0)[0] + global_offset[2][0]);

        Point3 tar_c1 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][1],
                (double) tvec.get(1, 0)[0] + global_offset[1][1], (double) tvec.get(2, 0)[0] + global_offset[2][1]);

        Point3 tar_c2 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][2],
                (double) tvec.get(1, 0)[0] + global_offset[1][2], (double) tvec.get(2, 0)[0] + global_offset[2][2]);

        Point3 tar_c3 = new Point3((double) tvec.get(0, 0)[0] + global_offset[0][3],
                (double) tvec.get(1, 0)[0] + global_offset[1][3], (double) tvec.get(2, 0)[0] + global_offset[2][3]);

        MatOfPoint3f offset_c0 = new MatOfPoint3f();
        offset_c0.fromArray(tar_c0);

        MatOfPoint3f offset_c1 = new MatOfPoint3f();
        offset_c1.fromArray(tar_c1);

        MatOfPoint3f offset_c2 = new MatOfPoint3f();
        offset_c2.fromArray(tar_c2);

        MatOfPoint3f offset_c3 = new MatOfPoint3f();
        offset_c3.fromArray(tar_c3);

        offset_c.add(offset_c0);
        offset_c.add(offset_c1);
        offset_c.add(offset_c2);
        offset_c.add(offset_c3);
    }

    private static double multiplyMatricesCell(double[][] firstMatrix, double[][] secondMatrix, int row, int col) {
        double cell = 0;
        for (int i = 0; i < secondMatrix.length; i++) {
            cell += firstMatrix[row][i] * secondMatrix[i][col];
        }
        return cell;
    }

    private static double[][] multiplyMatrices(double[][] firstMatrix, double[][] secondMatrix) {
        double[][] result = new double[firstMatrix.length][secondMatrix[0].length];

        for (int row = 0; row < result.length; row++) {
            for (int col = 0; col < result[row].length; col++) {
                result[row][col] = multiplyMatricesCell(firstMatrix, secondMatrix, row, col);
            }
        }
        return result;
    }

    public static Mat[] find_paper(Mat img, List<org.opencv.core.Point> src_pts, Mat originalImg, Rect target_rect) {

        List<Integer> list_x = new ArrayList<Integer>();
        List<Integer> list_y = new ArrayList<Integer>();

        for (int i = 0; i < 4; i++) {
            Imgproc.circle(img, src_pts.get(i), 5, new Scalar(255, 0, 0), -1);
            list_x.add((int) src_pts.get(i).x);
            list_y.add((int) src_pts.get(i).y);
        }
        Collections.sort(list_x);
        Collections.sort(list_y);

        double max_w = list_x.get(3) - list_x.get(0);
        double max_h = list_y.get(3) - list_y.get(0);
//	    1-------0
//	    |		|
//	    |  x,y  |
//	    |		|
//	    2-------3
        MatOfPoint2f dst_pts = new MatOfPoint2f(new org.opencv.core.Point(max_w - 1, 0), new org.opencv.core.Point(0, 0), new org.opencv.core.Point(0, max_h - 1),
                new org.opencv.core.Point(max_w - 1, max_h - 1));
        Log.i ("YourService","width : "+max_w+", height : "+max_h);
        MatOfPoint2f _pts = new MatOfPoint2f();
        _pts.fromList(src_pts);
        target_rect= new Rect();
        Mat cropped_img = cropped_ROI(originalImg, _pts, target_rect);

        Mat perspective_tf = Imgproc.getPerspectiveTransform(_pts, dst_pts);
        Mat warped_img = new Mat();
        Imgproc.warpPerspective(originalImg, warped_img, perspective_tf, new Size(max_w, max_h), Imgproc.INTER_LINEAR);
        Mat[] ret={warped_img, cropped_img};
        return ret;
    }

    public static Mat cropped_ROI(Mat img, MatOfPoint2f _pts, Rect target_rect) {
        target_rect = Imgproc.boundingRect(_pts);
        Mat cropped_img;
        cropped_img = img.submat(target_rect);
        return cropped_img;
    }


}

