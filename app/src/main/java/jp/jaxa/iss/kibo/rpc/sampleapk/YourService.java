package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
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

        Point currentPoint ;  //storing the co-ordinates where Astrobee is placed

        api.judgeSendStart();
        for(int i=0;i<6;i++){
            try{
                currentPoint = api.getTrustedRobotKinematics().getPosition();
                position = moveToPos(currentPoint,posX[i], posY[i], posZ[i], quarX[i], quarY[i], quarZ[i], quarW[i]);
                Log.i(TAG,"Found something : "+position);
                api.judgeSendDiscoveredQR(i, position);
                String list[] = position.split(",");   //position is of format pos_x, 3.15
                posP3[i] = Double.parseDouble(list[1]);


            }catch(Exception ex){
                Log.e(TAG,"Error at "+ i + "th position : "+ex.getMessage());
            }
        }


        try{
            currentPoint = api.getTrustedRobotKinematics().getPosition();
            api.laserControl(true);
            moveToPos(currentPoint,posP3[0], posP3[1], posP3[2], posP3[3], posP3[4], posP3[5], 0.7071068);
            Log.i(TAG,"Moved to P3");
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

    // You can add your method


    private String moveToPos(Point currentPoint,double pos_x, double pos_y, double pos_z,
                                 double qua_x, double qua_y, double qua_z,
                                 double qua_w) {

        final int LOOP_MAX = 5;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);


        for(int i = flag_obstacle;i<KOZ.length;i++){
            double currentX = currentPoint.getX();
            double currentY = currentPoint.getY();
            double currentZ = currentPoint.getZ();
            //robot moves in the negative y direction, KOZ[][1][1] has the first y co-ordinates of obstacles
            if(currentY>=KOZ[i][1][1] && pos_y<=KOZ[i][1][1]){

                Point obstacle_min = new Point(KOZ[i][0][0],KOZ[i][1][0],KOZ[i][2][0]);
                Point obstacle_max = new Point(KOZ[i][0][1],KOZ[i][1][1],KOZ[i][2][1]);

                double slopeX = (currentX - pos_x)/(currentY - pos_y);

                double obsStartX = trajectoryLine(slopeX,currentX,currentY,obstacle_max.getY()); //calculate the x value at the start of the obstacle in the trajectory line

                if(obsStartX<obstacle_min.getX() && pos_x<obstacle_min.getX()){
                    Log.i(TAG,i+"th Obstacle avoided by the trajectory crossing left to the obstacle");
                    continue;
                }

                else if(obsStartX>obstacle_max.getX() && pos_x>obstacle_max.getX()){
                    Log.i(TAG,i+"th Obstacle avoided by the trajectory crossing right to the obstacle");
                    continue;
                }

                double slopeZ = (currentZ - pos_z)/(currentY - pos_y);

                double obsStartZ = trajectoryLine(slopeZ,currentZ,currentY,obstacle_max.getY()); //calculate the z value at the start of the obstacle in the trajectory line

                if(obsStartZ<obstacle_min.getZ() && pos_z<obstacle_min.getZ()){
                    Log.i(TAG,i+"th Obstacle avoided by the trajectory going above the obstacle");
                    continue;
                }
                else if(obsStartZ>obstacle_max.getZ() && pos_z>obstacle_max.getZ()){
                    Log.i(TAG,i+"th Obstacle avoided by the trajectory going below the obstacle");
                    continue;
                }

                Log.i(TAG,"Collided with Keep Out Zone: "+ i);

                try {
                    currentPoint = obstacle(currentPoint,obstacle_min, obstacle_max, point);
                    Log.i(TAG,"Avoided Obstacle:"+i+ " Position: " +currentPoint.toString());
                    flag_obstacle =i + 1; //crossed ith obstacle, next time iteration starts from i+1 obstacle
                }catch(Exception ex){
                    Log.e(TAG, "Failed at avoiding obstacle : "+ ex.getMessage());
                }




            }
        }


        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
        Log.i(TAG,"QR code Position : "+point.toString());
        Bitmap snapshot = api.getBitmapNavCam();
        try{
            return readQRCode(snapshot);

        }catch(Exception ex){
            Log.e(TAG, "Error while reading QR code: "+ex.getMessage());
            return null;
        }

    }


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


    public Point obstacle(Point point,Point obstacle_min, Point obstacle_max,Point target){
        double currentZ = point.getZ();
        double currentY = point.getY();
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

        currentY =obstacle_min.getY() + 0.22; //getting infront of the obstacle

        Point newpoint = new Point(currentX,currentY,currentZ);
        Quaternion currentQuarter  = new Quaternion(0,0,(float)0,(float)0);
        Result result = api.moveTo(newpoint, currentQuarter, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < 3){
            result = api.moveTo(newpoint, currentQuarter, true);
            ++loopCounter;
        }

        Log.i(TAG,"Position : "+newpoint.toString());

        currentY =obstacle_min.getY() - 0.32; //y is negative always and crossing the obstacle
        newpoint = new Point(currentX,currentY,currentZ);

        result = api.moveTo(newpoint, currentQuarter, true);

        loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < 3){
            result = api.moveTo(newpoint, currentQuarter, true);
            ++loopCounter;
        }

        return newpoint;


    }


    public double trajectoryLine(double slope, double startx, double starty, double endy){
        return startx + slope*(endy - starty);
    }

}

