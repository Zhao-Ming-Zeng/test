package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class YourService extends KiboRpcService {
   private final String TAG = this.getClass().getSimpleName();
    @Override
    protected void runPlan1(){
        Log.i(TAG,"start mission");

        // The mission starts.
        api.startMission();


        // Move to a point.
        Point point = new Point(10.9d, -9.92884d, 5.195d);
        Quaternion quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Result result =api.moveTo(point, quaternion, false);
        /*//two
        point = new Point(10.95d,-10.58d,5.195d);
        quaternion =new Quaternion(0f,0f,0f,1f);//90
        api.moveTo(point, quaternion, false);
        //three
        point = new Point(10.925d,-8.875d,3.76203d);
        quaternion =new Quaternion(0f,0f,0f,1f);//90
        api.moveTo(point, quaternion, false);
        //four
        point = new Point(10.925d,-7.925d,3.76093d);
        quaternion =new Quaternion(0f,0f,0f,1f);//90
        api.moveTo(point, quaternion, false);
        //five
        point = new Point(9.866984d,-6.8525d,4.945d);
        quaternion =new Quaternion(0f,0f,0f,1f);//90
        api.moveTo(point, quaternion, false);*/

        int loopCounter=0;
        int loopMax = 5;
        Mat image=new Mat();
        List<Mat> corner = new ArrayList<>();
        Mat markerIds = new Mat();
        while(!result.hasSucceeded()&& loopCounter < loopMax)
        {
            //retry
            result=api.moveTo(point,quaternion,true);
            ++loopCounter;
        }

        // Get a camera image.
        image = api.getMatNavCam();
        //Save the image
        api.saveMatImage(image,"one.png");
        api.saveMatImage(image,"two.png");


        /* ******************************************************************************** */
        /* Write your code to recognize the type and number of landmark items in each area! */
        /* If there is a treasure item, remember it.                                        */
        /* ******************************************************************************** */
        //Detect AR
        Dictionary dictionary=Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners=new ArrayList<>();
        Aruco.detectMarkers(image,dictionary,corners,markerIds);
        //Check ARs
        if(corners.isEmpty()){
            Log.w(TAG,"Cannot detect ARs.");
            loopCounter++;
        } else {
           // break;
        }
        //Get camera matrix
        Mat cameraMatrix =new Mat (3,3,CvType.CV_64F);
        cameraMatrix.put(0,0,api.getNavCamIntrinsics()[0]);
        //Get Lens distortion parameters
        Mat cameraCoefficients =new Mat (3,3,CvType.CV_64F);
        cameraCoefficients.put(0,0,api.getNavCamIntrinsics()[1]);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);
        //Undistort image
        Mat undistorImg = new Mat();
        Calib3d.undistort(image,undistorImg,cameraMatrix,cameraCoefficients);

        //pattern matching
        //Load template images
        Mat[] templates = new Mat[item_template_images.length()];
        for(int i=0;i<TEMPLATE_FILE_NAME.length;i++){
                try {
                    //open the template image file in Bitmap from the file name and convert to Mat
                    InputStream inputStream = getAssets().open(TEMPLATE_FILE_NAME[i]);
                    Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                    Mat mat = new Mat();
                    Utils.bitmapToMat(bitmap, mat);
                    //Convert to grayscale
                    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);
                    //Assign to an array of temmplates
                    templates[i] = mat;
                    inputStream.close();
                }catch (IOException e){
                    e.printStackTrace();
                }
        }
        //Number of matches for each template
        int templateMatchCnt[]=new int[10];
        //Get the number of template matches
        for (int tempNum=0;tempNum<templates.length;tempNum++){
            //Number of matches
            int matchCnt=0;
            //Coordinates of the matched location
            List<org.opencv.core.Point> matched = new ArrayList<>();
            //loading template image and terget image
            Mat template = templates[tempNum].clone();
            Mat targetImg = undistorImg.clone();

            //Pattern mathing
            int widthMin =20;//[px]
            int widthMax=100;//[px]
            int changeWidth =5;//[px]
            int changeAngle=45;//[degree]

            for(int i=widthMin;i<=widthMax;i+=changeAngle){
                for(int j=0;j<=360;j+=changeAngle){
                    Mat resizedTemp = resizeImg(template,i);
                    Mat rotResizedTemp = rotImg(resizedTemp,j);

                    Mat result = new Mat();
                    Imgproc.matchTemplate(targetImg,rotResizedTemp,result,Imgproc.TM_CCOEFF_NORMED);
                }
            }
            //Get coordinates with similarity grater than or equal to the threshold
            double threshold = 0.8;
            Core.MinMaxLocResult mmlr  = Core.minMaxLoc(result);
            double macVal=mmlr.maxVal;
            if(maxVal >= threshold){
                Mat thresholdedResult = new Mat();
                Imgproc.threshold(result,thresholdedResult,threshold,1.0,Imgproc.THRESH_TOZERO);
                //Get match counters
                for (int y=0;y<thresholdedResult.rows();y++){
                    for (int x=0;x<thresholdedResult.rows();x++){
                        if(thresholdedResult.get(y,x)[0]>0){
                            matches.add(new org.opencv.core.Point(x,y));
                        }
                    }
                }
            }
        }
        //Avoid detecting the same location multiple times
        List<org.opencv.core.Point>filteredMatches = removeDuplicates(matches);
        matchCnt +=filteredMatches.size();
        //Number of matches for each template
        templateMatchCnt[tempNum] = matchCnt;

        // When you recognize landmark items, let’s set the type and number.
        int mostMatchTemplateNum = getMaxIndex(templateMatchCnt);
        api.setAreaInfo(1, TEMPLATE_NAME[mostMatchTemplateNum], templateMatchCnt[mostMatchTemplateNum]);

        /* **************************************************** */
        /* Let's move to each area and recognize the items. */
        /* **************************************************** */

        // When you move to the front of the astronaut, report the rounding completion.
        point = new Point(11.143d, -6.7607d, 4.9654d);
        quaternion = new Quaternion(0f, 0f, 0.707f, 0.707f);
        api.moveTo(point, quaternion, false);
        api.reportRoundingCompletion();

        /* ********************************************************** */
        /* Write your code to recognize which target item the astronaut has. */
        /* ********************************************************** */





        //Pattern matching
        //Load template images


        // Let's notify the astronaut when you recognize it.
        api.notifyRecognitionItem();

        /* ******************************************************************************************************* */
        /* Write your code to move Astrobee to the location of the target item (what the astronaut is looking for) */
        /* ******************************************************************************************************* */

        // Take a snapshot of the target item.
        api.takeTargetItemSnapshot();
    }

    @Override
    protected void runPlan2(){
       // write your plan 2 here.
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here.
    }

    // You can add your method.
    private String yourMethod(){
        return "your method";
    }

    private Mat resizeImg(Mat img,int width){
        int height = (int)(img.row()*((double)width/img.cols()));
        Mat resizeImg = new Mat();
        Imgproc.resize(img,resizeImg,new Size(width,height));
         return resizeImg;
    }
    private  Mat rotImg(Mat img,int angle){
        org.opencv.core.Point center = new org.opencv.core.Point(img.cols()/2.0,img.rows()/2.0);
        Mat rotateMat = Imgproc.getRotationMatrix2D(center,angle,1.0);
        Mat rotatedImg = new Mat();
        Imgproc.warpAffine(img,rotatedImg,rotateMat,img.size());
        return rotatedImg;
    }
    //Remove multiple detections
    private static List<org.opencv.core.Point> removeDuplicates (List<org.opencv.core.Point> points){
        double length =10;
        List<org.opencv.core.Point> filteredList = new ArrayList<>();
        for(org.opencv.core.Point point:points){
            boolean isInclude = false;
            for(org.opencv.core.Point checkPoint : filteredList){
                double distance = calculateDistance(point,checkPoint);
                if(distance <= length){
                    isInclude = true;
                    break;
                }
            }
            if(!isInclude){
                filteredList.add(point);
            }
        }
        return filteredList;
    }
   //Find the distance between two points
    private static double calculateDistance(org.opencv.core.Point p1,org.opencv.core.Point p2){
        double dx =p1.x - p2.x;
        double dy =p1.y - p2.y;
        return  Math.sqrt(Math.pow(dx,2)+Math.pow(dy,2));
    }
    //Get the maximum value of an array
    private  int getMaxIndex(int[] array){
        int max = 0;
        int maxIndex = 0;
        //Find the index of the element with the largest value
        for(int i = 0 ; i < array.length; i++){
            if(array[i]>max){
                if(array[i] > max){
                    max = array[i];
                    maxIndex=i;
                }
            }
            return maxIndex;
        }

    }

}
