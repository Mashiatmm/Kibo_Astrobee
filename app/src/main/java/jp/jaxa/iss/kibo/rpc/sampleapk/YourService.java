package jp.jaxa.iss.kibo.rpc.sampleapk;


import android.graphics.Bitmap;
import android.icu.util.RangeValueIterator;
import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;


import com.google.zxing.BinaryBitmap;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;

import com.google.zxing.common.HybridBinarizer;

import static java.lang.Math.abs;
import static org.opencv.core.CvType.CV_8UC1;

import java.lang.Math;
import java.util.ArrayList;
import java.util.List;


import org.opencv.aruco.Aruco;

import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    private static final String    TAG = "KiboFiendFyre";

    int flag_obstacle = 0;//keeping track of each obstacle crossed

    //Keep In Zone Co - ordinates
    private double [] KIZ_x = {10.25,11.65};
    private double [] KIZ_y = {-9.75,-3};
    private double [] KIZ_z = {4.2,5.6};

    //Keep Out Zone Co-ordinates
    private double KOZ[][][] ={
        {
            {10.75,10.95},{-4.9,-4.7},{4.8,5.0}
        },
        {
            {10.75,11.95},{-6.5 ,-6.4},{3.9,5.9}
        },
        {
            {9.95,10.85},{-7.2,-7.1},{3.9,5.9}
        },
        {
            {10.10,11.1},{-8.6,-8.3},{5.4,5.9}
        },
        {
            {11.45,11.95},{-9.0,-8.5},{4.1,5.1}
        },
        {
            {9.95,10.45},{-9.1,-8.6},{4.6,5.6}
        },
        {
            {10.95,11.15},{-8.4,-8.2},{4.9,5.1}
        },
        {
            {11.05,11.25},{-8.9,-8.7},{4.2,4.4}
        },
        {
            {10.45,10.65},{-9.1,-8.9},{4.6,4.8}
        }

    };


    @Override
    protected void runPlan1(){
        String position;
        double [] posP3 = new double[6]; //for storing the co-ordinates of P3
        //Astrobee 1 ft cube  = 0.3048 meter per side, half approx = 0.16 m, diagonally half length = 0.22 m
        //the below 7 arrays constitute the position of P1-1 to P2-3
        double []posX = {11.5,11,11,10.30,11.5,11};//4th value 10.30 ....10.25 + 0.22 = 10.47
        double []posY = {-5.7,-6,-5.5,-7.5,-8,-7.7};
        double []posZ = {4.5,5.55,4.33,4.7,5,5.55}; //2nd value and 6th value = 5.55 KIZ_lim = 5.6 - 0.22 = 5.42
        double []quarX = {0,0,0,0,0,0};
        double []quarY = {0, -0.7071068, 0.7071068,0,0, -0.7071068};
        double []quarZ = {0,0,0,1,0,0};
        double []quarW = {1, 0.7071068, 0.7071068,0,1, 0.7071068};

        api.judgeSendStart();
        for(int i=0;i<6;i++){
            try{
                position = moveToPos(posX[i], posY[i], posZ[i], quarX[i], quarY[i], quarZ[i], quarW[i]);
                Log.i(TAG,"Found something : "+position);
                api.judgeSendDiscoveredQR(i, position);
                String list[] = position.split(",");   //position is of format pos_x, 3.15
                posP3[i] = Double.parseDouble(list[1]);


            }catch(Exception ex){
                Log.e(TAG,"Error at "+ i + "th position : "+ex.getMessage());
            }
        }


        try{
            String ARMarker = moveToP3(posP3[0], posP3[1], posP3[2], posP3[3], posP3[4], posP3[5]);
            Log.i(TAG,"Moved to P3: "+ARMarker);
            api.judgeSendDiscoveredAR(ARMarker);
            api.laserControl(true);
            api.judgeSendFinishSimulation();
        }
        catch(Exception ex){
            Log.e(TAG,"Error at P3");
        }

    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }



    //MOVE TO P1-1 TO P2-3
    private String moveToPos(double pos_x, double pos_y, double pos_z,
                                 double qua_x, double qua_y, double qua_z,
                                 double qua_w) {


        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);

        //check for collision with KOZ
        for(int i = flag_obstacle;i<KOZ.length;i++){
            Point currentPoint = api.getTrustedRobotKinematics().getPosition();

            double currentY = currentPoint.getY();

            if(pos_y > KOZ[i][1][1]) break; //If obstacles come after the target then break the loop
            //robot moves in the negative y direction, KOZ[][1][1] has the first y co-ordinates of obstacles
            if(currentY>=KOZ[i][1][1] && pos_y<=KOZ[i][1][1]){

                int flag = checkForCollision(i,currentPoint,point);
                if(flag == 1){continue;}

                Log.i(TAG,"Collided with Keep Out Zone: "+ i);

                try {
                    Point obstacle_min = new Point(KOZ[i][0][0],KOZ[i][1][0],KOZ[i][2][0]);
                    Point obstacle_max = new Point(KOZ[i][0][1],KOZ[i][1][1],KOZ[i][2][1]);

                    currentPoint = obstacle(currentPoint,obstacle_min, obstacle_max, point);
                    Log.i(TAG,"Avoided Obstacle:"+i+ " Position: " +currentPoint.toString());
                    flag_obstacle =i + 1; //crossed ith obstacle, next time iteration starts from i+1 obstacle
                }catch(Exception ex){
                    Log.e(TAG, "Failed at avoiding obstacle : "+ ex.getMessage());
                }


            }
        }

        //Move to the point
        moveBetweenPoints(point, quaternion,5);


        //Read QR Code
        Log.i(TAG,"QR code Position : "+point.toString());
        Bitmap snapshot = api.getBitmapNavCam();
        try{
            return readQRCode(snapshot);

        }catch(Exception ex){
            Log.e(TAG, "Error while reading QR code: "+ex.getMessage());
            return null;
        }

    }


    //MOVING TO P3 AND READING AR TAG
    private String moveToP3(double pos_x, double pos_y, double pos_z,
                             double qua_x, double qua_y, double qua_z) {


        final Point point = new Point(pos_x, pos_y, pos_z);
        float qua_w = (float) Math.sqrt(1 - (Math.pow(qua_x,2)+Math.pow(qua_y,2)+Math.pow(qua_z,2)));
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, qua_w);


        // AVOIDING OBSTACLES
        for(int i = flag_obstacle;i<KOZ.length;i++){
            Point currentPoint = api.getTrustedRobotKinematics().getPosition();

            double currentY = currentPoint.getY();

            if(pos_y > KOZ[i][1][1]) break; //If obstacles come after the target then break the loop
            //robot moves in the negative y direction, KOZ[][1][1] has the first y co-ordinates of obstacles
            if(currentY>=KOZ[i][1][1] && pos_y<=KOZ[i][1][1]){

                int flag = checkForCollision(i,currentPoint,point);
                if(flag == 1){continue;}

                Log.i(TAG,"Collided with Keep Out Zone: "+ i);

                try {
                    Point obstacle_min = new Point(KOZ[i][0][0],KOZ[i][1][0],KOZ[i][2][0]);
                    Point obstacle_max = new Point(KOZ[i][0][1],KOZ[i][1][1],KOZ[i][2][1]);

                    currentPoint = obstacle(currentPoint,obstacle_min, obstacle_max, point);
                    Log.i(TAG,"Avoided Obstacle:"+i+ " Position: " +currentPoint.toString());
                    flag_obstacle =i + 1; //crossed ith obstacle, next time iteration starts from i+1 obstacle
                }catch(Exception ex){
                    Log.e(TAG, "Failed at avoiding obstacle : "+ ex.getMessage());
                }


            }
        }


        String ARMarker = null;

        //Move to P3 Position
        moveBetweenPoints(point,quaternion,5);

        Log.i(TAG,"AR Position : "+point.toString()+" Quaternion: "+quaternion.toString());

        //Read AR Marker
        try{
        //Log.i(TAG,snapshot.toString());
            int count = 0;
            while(ARMarker == null){
                ARMarker = readArucoMarker();
                count ++;
                if(count == 5) break;
            }

        }catch(Exception ex){
            Log.e(TAG, "Error while reading AR Tag: "+ex.getMessage());
        }

        return ARMarker;



    }

    //READ QR TAGS
    public static String readQRCode(Bitmap snapshot)
            throws  NotFoundException {

        int[] intArray = new int[snapshot.getWidth() * snapshot.getHeight()];
        snapshot.getPixels(intArray, 0, snapshot.getWidth(), 0, 0, snapshot.getWidth(),
                snapshot.getHeight());
        LuminanceSource source = new RGBLuminanceSource(snapshot.getWidth(),
                snapshot.getHeight(), intArray);
        BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
        com.google.zxing.Result qrCodeResult = new MultiFormatReader().decode(bitmap);
        return qrCodeResult.getText();
    }

    //READ ARUCO TAGS
    public String readArucoMarker() {

        Mat snapshot = api.getMatNavCam();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat arucoIDs = new Mat();

        try {
            Log.i(TAG,"Calling Aruco Detect Markers");
            Aruco.detectMarkers(snapshot, dictionary, corners, arucoIDs);
            Log.i(TAG,"Detected Markers ");
            Log.i(TAG, "ArucoIds : " + arucoIDs.toString());
            Log.i(TAG, "Corners : " + corners.toString());

            double[] id = arucoIDs.get(0,0);
            Log.i(TAG, String.valueOf(id));
            double camera_matrix[] = {344.173397, 0.000000, 630.793795,
                    0.000000, 344.277922, 487.033834,
                    0.000000, 0.000000, 1.000000};
            Mat CamMatrix = new Mat(3, 3, CV_8UC1);
            CamMatrix.put(3, 3, camera_matrix);

            double dist_coeffs[] = {-0.152963, 0.017530, -0.001107, -0.000210, 0.000000};
            Mat DistCoeffs = new Mat(1, 5, CV_8UC1);
            DistCoeffs.put(1, 5, dist_coeffs);

            Mat rvecs = new Mat();
            Mat tvecs = new Mat();
            Mat objPoints = new Mat();

            Aruco.estimatePoseSingleMarkers(corners, (float) 0.05, CamMatrix, DistCoeffs, rvecs, tvecs, objPoints);
            Log.i(TAG, "rvecs: " + rvecs.toString());
            Log.i(TAG, "tvecs: " + tvecs.toString());
            Log.i(TAG, "ObjPoints : " + objPoints);

            //Aruco.drawAxis(snapshot, CamMatrix, DistCoeffs, rvecs, tvecs, 0.1f);

            return String.valueOf((int)id[0]);

        }catch(Exception e){
            Log.e(TAG,"error: " + e.getMessage());
        }
        return null;
    }


    //Cross Obstacle
    public Point obstacle(Point point,Point obstacle_min, Point obstacle_max,Point target){
        double currentZ = point.getZ();
        double currentX = point.getX();


        double x_min = obstacle_min.getX();
        double x_max = obstacle_max.getX();
        double z_min = obstacle_min.getZ();
        double z_max = obstacle_max.getZ();

        //0.22 being the highest diagonal length
        double posX = target.getX();
        if (abs(posX - x_min) <= abs(posX - x_max) && x_min - 0.22 >KIZ_x[0]){
            currentX = x_min - 0.22;
        }
        else if(x_max + 0.22 <KIZ_x[1]){
            currentX = x_max + 0.22;
        }

        double posZ = target.getZ();
        if(abs(posZ - z_min) <= abs(posZ - z_max) && z_min - 0.22>KIZ_z[0]){
            currentZ = z_min - 0.22;
        }
        else if(z_max + 0.22<KIZ_z[1]){
            currentZ = z_max + 0.22;
        }

        double currentY =obstacle_min.getY() + 0.22; //getting infront of the obstacle

        Point newpoint = new Point(currentX,currentY,currentZ);
        Quaternion currentQuarter  = new Quaternion(0,0,(float)0,(float)0);
        moveBetweenPoints(newpoint,currentQuarter,3);
        
        Log.i(TAG,"Obstacle : Position : "+newpoint.toString());

        currentY =obstacle_min.getY() - 0.32; //y is negative always and crossing the obstacle
        newpoint = new Point(currentX,currentY,currentZ);

        moveBetweenPoints(newpoint,currentQuarter,3);
        

        return newpoint;


    }

    //CALCULATE VALUE OF A LINE
    public double trajectoryLine(double slope, double startx, double starty, double endy){
        return startx + slope*(endy - starty);
    }


    //Check is a collision happens with any obstacle
    public int checkForCollision(int i,Point currentPoint, Point point){

        double currentX = currentPoint.getX();
        double currentY = currentPoint.getY();
        double currentZ = currentPoint.getZ();

        double pos_x = point.getX();
        double pos_y = point.getY();
        double pos_z = point.getZ();

        Point obstacle_min = new Point(KOZ[i][0][0],KOZ[i][1][0],KOZ[i][2][0]);
        Point obstacle_max = new Point(KOZ[i][0][1],KOZ[i][1][1],KOZ[i][2][1]);

        double slopeX = (currentX - pos_x)/(currentY - pos_y);
        double obsStartX = trajectoryLine(slopeX,currentX,currentY,obstacle_max.getY()); //calculate the x value at the start of the obstacle in the trajectory line
        double obsEndX = trajectoryLine(slopeX,currentX,currentY,obstacle_min.getY());

        double slopeZ = (currentZ - pos_z)/(currentY - pos_y);
        double obsStartZ = trajectoryLine(slopeZ,currentZ,currentY,obstacle_max.getY()); //calculate the z value at the start of the obstacle in the trajectory line
        double obsEndZ = trajectoryLine(slopeZ,currentZ,currentY,obstacle_min.getY());

        Point newpoint = new Point(obsEndX,obstacle_min.getY(),obsEndZ); // move to the endpoint of obstacle
        Quaternion currentQuarter  = new Quaternion(0,0,(float)0,(float)0);

        if(obsStartX + 0.16<obstacle_min.getX() && pos_x<obstacle_min.getX()){
            moveBetweenPoints(newpoint,currentQuarter,3);
            Log.i(TAG,i+"th Obstacle avoided by the trajectory crossing left to the obstacle");
            return 1;
        }

        else if(obsStartX - 0.16>obstacle_max.getX() && pos_x>obstacle_max.getX()){
            moveBetweenPoints(newpoint,currentQuarter,3);
            Log.i(TAG,i+"th Obstacle avoided by the trajectory crossing right to the obstacle");
            return 1;
        }

        else if(obsStartZ + 0.16 <obstacle_min.getZ() && pos_z<obstacle_min.getZ()){
            moveBetweenPoints(newpoint,currentQuarter,3);
            Log.i(TAG,i+"th Obstacle avoided by the trajectory going above the obstacle");
            return 1;
        }
        else if(obsStartZ - 0.16>obstacle_max.getZ() && pos_z>obstacle_max.getZ()){
            moveBetweenPoints(newpoint,currentQuarter,3);
            Log.i(TAG,i+"th Obstacle avoided by the trajectory going below the obstacle");
            return 1;
        }

        return 0;
    }

    //MOVE BETWEEN TWO POINTS
    public void moveBetweenPoints(Point newpoint , Quaternion currentQuarter, int LOOPMAX){
        Result result = api.moveTo(newpoint, currentQuarter, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < LOOPMAX){
            result = api.moveTo(newpoint, currentQuarter, true);
            ++loopCounter;
        }

        Log.i(TAG,"MoveBetweenPoints: Position : "+newpoint.toString());
    }



}

