package org.firstinspires.ftc.teamcode.SkyStone.Tests;

import android.util.Log;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.CbColorFilter;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;

import java.util.ArrayList;
import java.util.List;

public class OpenCV extends DogeCVDetector {
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer

    //Create the default filters and scorers
    public DogeCVColorFilter blackFilter = new GrayscaleFilter(0, 25);
    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 70); //Default Yellow blackFilter

    public RatioScorer ratioScorer = new RatioScorer(1.25, 3); // Used to find the short face of the stone
    public MaxAreaScorer maxAreaScorer = new MaxAreaScorer( 0.01);                    // Used to find largest objects
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05); // Used to find objects near a tuned area value


    // Results of the detector
    private Point screenPosition = new Point(); // Screen position of the mineral
    private Rect foundRect = new Rect(); // Found rect


    private Point matchLoc;



    private Mat rawImage = new Mat();
    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();
    private Mat blackMask = new Mat();
    private Mat yellowMask = new Mat();
    private Mat hierarchy  = new Mat();
    private Mat workTemp = new Mat();
    private Mat templateImg = Imgcodecs.imread("C:\\Users\\Alek\\Documents\\GitCode\\SkyStone\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\SkyStone\\Tests\\Images\\size.PNG");
    private Mat edge = new Mat();

    private Mat matchRes = new Mat();

    public Point getScreenPosition() {
        return screenPosition;
    }



    public Rect foundRectangle() {
        return foundRect;
    }


    public OpenCV() {
        detectorName = "Skystone Detector";
    }

    @Override
    public Mat process(Mat input) {
        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(displayMat);
        input.copyTo(blackMask);
        templateImg.copyTo(workTemp);


        //Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        //Imgproc.Canny(workingMat, edge,50,150);
        //Imgcodecs.imwrite("Images", edge);

        int w= templateImg.width();
        int h = templateImg.height();


        if (templateImg.type() == workingMat.type()) {
            Imgproc.matchTemplate(workingMat, templateImg, matchRes, Imgproc.TM_SQDIFF);
            Core.MinMaxLocResult mmr = Core.minMaxLoc(matchRes);

            matchLoc = mmr.minLoc;
        }

        return edge;
    }

    public Point getResult(){
        return matchLoc;
    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorer);

        // Add diffrent scorers depending on the selected mode
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }
    }
}
