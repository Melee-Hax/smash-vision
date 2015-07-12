#include "ObjectTracker.h"
#include <list>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <tesseract/baseapi.h>
/*
 ********FUNCTIONS ARE REMNANTS OF THE KEYPOINT MATCHING METHOD. MAY NEED TO FALL BACK ON THIS FOR SCALE INVARIANT MATCHING , SO LEAVING IN********
 
std::vector<cv::KeyPoint> detectKeypoints( int minHessian, cv::Mat img , std::vector<cv::Mat> mask , bool debug = false);

cv::Mat getDescriptors( int minHessian, cv::Mat img , std::vector<cv::KeyPoint> keypoints , bool debug = false);

std::vector< cv::DMatch > match( cv::Mat descriptors_object , cv::Mat descriptors_scene , bool debug = false);

cv::Mat localize( cv::Mat img_object , std::vector<cv::KeyPoint> keypoints_object , cv::Mat img_scene , std::vector<cv::KeyPoint> keypoints_scene , std::vector< cv::DMatch > matches, bool debug = false);
*/
using namespace cv;
using namespace std;


int errorFlag = 0;                  //error flag
const int STARTFRAME = 160;         //what frame should we start on?
float avgDistance;                  //should we find a match, how good is it?
float THRESHOLD = .3;         //minimum acceptable template match accuracy
auto cwd = boost::filesystem::current_path();

pair<double,cv::Point>* templateMatch(cv::Mat img, cv::Mat templ, bool relaxed = false, float thresh = THRESHOLD);
void getScale(cv::Mat);
int readCountdown(cv::Mat scene);
int getCurrentTime(cv::Mat scene);
void setUpNumbers(cv::Mat scene);
bool compPoints (cv::Point i,cv::Point j);
pair<string, string> getMatchup(Mat scene);

float videoScale;
float game_start_time;

Mat Zero;
Mat One;
Mat Two;
Mat Three;
Mat Four;
Mat Five;
Mat Six;
Mat Seven;
Mat Eight;
Mat Nine;

int main(int argc, char **argv)
{
    string filename = cwd.string() + "/Resources/match.mp4";
    
    vision::ObjectTracker tracker(filename);            //init tracker, will give us img_scene values
    
    Mat img_object = imread(cwd.string() + "/Resources/eight.png",1);
    Mat img_scene = tracker.getCurrentFrame();      //img_scene points to first frame
    
    while(tracker.getCurrentFrameNumber() < STARTFRAME){//loop until you get to the frame you're interested in
        tracker.nextFrame();
    }
    
    setUpNumbers(img_scene);//can't set up numbers before game screen loaded
    //HOW DO WE FIGURE OUT WHEN THE GAME SCREEN IS UP?
    
    namedWindow("Frame " + to_string(tracker.getCurrentFrameNumber()));
    
    pair<double,Point> matchPoint;
    
    while(true) //iterate over frames
    {
        /*
		//Tesseract testing
		Rect roi = Rect(0, 0, img_scene.cols, img_scene.rows/3);
		Mat img_scene_roi = img_scene(roi);
		Mat gray;
		cvtColor(img_scene_roi, gray, CV_BGR2GRAY);

		tesseract::TessBaseAPI tess;
		tess.Init(NULL, "eng", tesseract::OEM_DEFAULT);
		tess.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);
		tess.SetVariable("tessedit_char_whitelist", "0123456789");
		tess.SetImage((uchar*)gray.data, gray.cols, gray.rows, 1, gray.cols);


        imshow("Frame " + to_string(tracker.getCurrentFrameNumber()), img_scene_roi );
        destroyWindow("Frame " + to_string(tracker.getCurrentFrameNumber()-1));
        matchPoint = *templateMatch(img_scene, img_object);
         

		char* text = tess.GetUTF8Text();
		cout << text << endl;
         */
		/* float time = getCurrentTime(img_scene); */
		/* cout << time << endl; */
        pair<string,string> matchup = getMatchup(img_scene);
        if(matchup.second != ""){
            cout << matchup.first << " vs. " << matchup.second;
            waitKey(0);
        }
        
        if(matchPoint.first > 0){
            //display result
            rectangle(img_scene, matchPoint.second, Point( matchPoint.second.x + img_object.cols , matchPoint.second.y + img_object.rows ), Scalar::all(255), 2, 8, 0 );
            
            namedWindow("Match: " + to_string((1-matchPoint.first)*100) + "% certainty"); //this whole % certainty thing is probably BS, just want to display match accuracy in percent form to make it human-readable
            imshow( "Match: " + to_string((1-matchPoint.first)*100) + "% certainty", img_scene );
            /* waitKey(0); */
            destroyWindow("Match: "+ to_string((1-matchPoint.first)*100) + "% certainty" );
        }
        cout << "Next frame...(" << tracker.getCurrentFrameNumber() << ")..." << endl;
        tracker.nextFrame();
        img_scene = tracker.getCurrentFrame();
        continue;
        
        /*
         ********ALL THE BELOW IS KEYPOINT MATCHING. MAY NEED TO FALL BACK ON THIS SOMEDAY, SO LEAVING IN********
        
        //-- Step 1: Detect the keypoints using SURF Detector
        std::vector<cv::KeyPoint> keypoints_object = detectKeypoints( 1 , img_object , ch );
        std::vector<cv::KeyPoint> keypoints_scene = detectKeypoints( 1 , img_scene , null);
        
        //cornerHarrisDemo(0, argv[1], 0); still exploring this method to make keypoints more robust
        
        switch(errorFlag){
            case 0 : break;
            case 1 :
                std::cout << "No keypoints found...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
        }
        if(errorFlag > 0){
            errorFlag = 0;
            tracker.nextFrame();
            img_scene = tracker.getCurrentFrame();
            continue;
        }
        
        //-- Step 2: Calculate descriptors using SURF extractor
        
        cv::Mat descriptors_object = getDescriptors( 1, img_object, keypoints_object  );
        cv::Mat descriptors_scene = getDescriptors( 1, img_scene, keypoints_scene  );
        
        switch(errorFlag){
            case 0 : break;
            case 1 :
                std::cout << "No descriptors found...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
        }
        if(errorFlag > 0){
            errorFlag = 0;
            tracker.nextFrame();
            img_scene = tracker.getCurrentFrame();
            continue;
        }
        
        //-- Step 3: Matching descriptor vectors using FLANN matcher
        
        std::vector< cv::DMatch > good_matches = match( descriptors_object, descriptors_scene , tracker.getCurrentFrameNumber() == 172 );
        
        
        switch(errorFlag){
            case 0 : break;
            case 1 :
                std::cout << "No matches found...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
            case 2:
                std::cout << "No good matches found...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
        }
        if(errorFlag > 0){
            errorFlag = 0;
            tracker.nextFrame();
            img_scene = tracker.getCurrentFrame();
            continue;
        }
        
        //Step 4: Localize matches, generate image
        cv::Mat img_matches = localize( img_object, keypoints_object, img_scene, keypoints_scene, good_matches );
        
        switch(errorFlag){
            case 0 : break;
            case 1 :
                std::cout << "OpenCV drawMatches bug...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
            case 2 :
                std::cout << "No homography found...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
            case 3 :
                std::cout << "I wasn't confident...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
                break;
        }
        if(errorFlag > 0){
            errorFlag = 0;
            tracker.nextFrame();
            img_scene = tracker.getCurrentFrame();
            continue;
        }
        
        if(avgDistance < .07) //arbitrary cutoff value. this is problematic because matches with worse homographies can have lower average distance.
        {
            //-- Show detected matches
            imshow( "Found a match", img_matches );
            printf("-- Average distance: %f \n", avgDistance );
            cv::waitKey(0);
        }
        
        avgDistance = 0;
        std::cout << "Next frame...(" << tracker.getCurrentFrameNumber() << ")" << std::endl;
        tracker.nextFrame();
        img_scene = tracker.getCurrentFrame();
        */
    }
    
    return 0;
}
/*
 ********FUNCTIONS ARE REMNANTS OF THE KEYPOINT MATCHING METHOD. MAY NEED TO FALL BACK ON THIS SOMEDAY, SO LEAVING IN********
 
std::vector<cv::KeyPoint> detectKeypoints( int minHessian, cv::Mat img , std::vector<cv::Mat> mask , bool debug){
    using namespace cv::xfeatures2d;
    cv::Ptr<SURF> detector = SURF::create( minHessian );
    using namespace cv;
    
    std::vector<KeyPoint> keypoints;
    
    if( mask.size() > 0 ){
        detector->detect( img, keypoints , mask[3] );
    }
    else{
        detector->detect( img, keypoints );
    }
    
    if(keypoints.size() == 0){
        errorFlag = 1;
    }
    
    if(debug){
        Mat tmp;
        cv::cvtColor(img , img , CV_RGBA2RGB);
        drawKeypoints( img, keypoints, tmp, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
        imshow( "keypoints", tmp );
        waitKey(0);
    }
    
    return keypoints;
}

cv::Mat getDescriptors( int minHessian, cv::Mat img , std::vector<cv::KeyPoint> keypoints , bool debug){
    using namespace cv::xfeatures2d;
    cv::Ptr<SURF> extractor = SURF::create( minHessian );
    using namespace cv;
    
    cv::Mat descriptors;
    extractor->compute( img, keypoints, descriptors );
    
    if(descriptors.size == 0){
        errorFlag = 1;
    }
    
    if(debug){
        imshow("descriptors" , descriptors);
        waitKey(0);
    }
    return descriptors;
}

std::vector< cv::DMatch > match( cv::Mat descriptors_object , cv::Mat descriptors_scene , bool debug){
    cv::FlannBasedMatcher matcher;
    
    std::vector< cv::DMatch > matches;
    
    matcher.match( descriptors_object, descriptors_scene, matches );
 
     //idea: increase accuracy of matches by only taking bidirectional matches
     //std::vector< DMatch > matchesforward;
     //matcher.match( descriptors_object, descriptors_scene, matchesforward );
     //std::vector< DMatch > matchesbackward;
     //matcher.match( descriptors_scene, descriptors_object, matchesbackward );
     //std::vector< DMatch > matches;
     //sort(matchesforward.begin(), matchesforward.end());
     //sort(matchesbackward.begin(), matchesbackward.end());
     //set_intersection(matchesforward.begin(),matchesforward.end(),matchesbackward.begin(),matchesbackward.end(),back_inserter(matches));
 
    
    if(matches.size() == 0){
        errorFlag = 1;
    }
    
    double max_dist = 0; double min_dist = 100;
    
    //-- get min and max distances between keypoints
    for( int i = 0; i < matches.size(); i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    
    std::vector< cv::DMatch > good_matches;
    
    for( int i = 0; i < descriptors_object.rows; i++ )
    {
        if( matches[i].distance < 2*min_dist )
        {
            good_matches.push_back( matches[i]);
            avgDistance += matches[i].distance;
        }
    }
    
    if(good_matches.size() == 0){
        avgDistance = 0;
        errorFlag = 2;
    }
    else{
        avgDistance = avgDistance/good_matches.size();
    }
    
    if(debug){
        //sorry dude, can't see matches until after localization
    }
    
    return good_matches;
}

cv::Mat localize( cv::Mat img_object , std::vector<cv::KeyPoint> keypoints_object , cv::Mat img_scene , std::vector<cv::KeyPoint> keypoints_scene , std::vector< cv::DMatch > good_matches, bool debug){
    cv::Mat temp,img_matches;
    img_object.convertTo(temp, img_scene.type());
    try{
        drawMatches( temp, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );} //draw the matches
    catch(cv::Exception){
        errorFlag = 1;
    }
    
    //-- Localize the object
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;
    
    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    
    cv::Mat H = findHomography( obj, scene, CV_LMEDS );
    
    //-- Get the corners from the object
    //can i use harriscorners here?
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
    obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
    std::vector<cv::Point2f> scene_corners(4);
    
    if(H.empty()){//sometimes there just isnt any kind of match at all
        errorFlag = 2;
        return H;
    }
    
    if(findFundamentalMat(obj, scene, cv::FM_RANSAC, 3, 0.99, cv::noArray()).empty()){//trying to use this to filter outliers
        errorFlag = 3;
        return H;
    }
    
    perspectiveTransform( obj_corners, scene_corners, H);
    
    IplImage tmp = img_matches;
    
    //-- Draw lines between the corners around the object in the scene
    cvLine( &tmp, scene_corners[0] + cv::Point2f( img_object.cols, 0), scene_corners[1] + cv::Point2f( img_object.cols, 0), cvScalar( 50, 100, 255), 4 );
    cvLine( &tmp, scene_corners[1] + cv::Point2f( img_object.cols, 0), scene_corners[2] + cv::Point2f( img_object.cols, 0), cvScalar( 50, 100, 255), 4 );
    cvLine( &tmp, scene_corners[2] + cv::Point2f( img_object.cols, 0), scene_corners[3] + cv::Point2f( img_object.cols, 0), cvScalar( 50, 100, 255), 4 );
    cvLine( &tmp, scene_corners[3] + cv::Point2f( img_object.cols, 0), scene_corners[0] + cv::Point2f( img_object.cols, 0), cvScalar( 50, 100, 255), 4 );
    
    if(debug){
        imshow("matches", img_matches );
        cv::waitKey(0);
    }
    
    return img_matches;
}
*/
pair<double,Point>* templateMatch(cv::Mat img, cv::Mat templ, bool relaxed, float thresh)
{
    using namespace std;
    using namespace cv;
    
    Mat result;
    int match_method = CV_TM_SQDIFF_NORMED;
    /// Source image to display
    Mat img_display;
    img.copyTo( img_display );
    
    /// Create the result matrix
    int result_cols =  img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;
    
    result.create( result_rows, result_cols, CV_32FC1 );
    
    /// Do the Matching and Normalize
    matchTemplate( img, templ, result, match_method );
    /* normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() ); */
    
    /// Localizing the best match with minMaxLoc
    double minVal = 0; Point minLoc = *new Point;
    Point matchLoc;
    
    minMaxLoc( result, &minVal, NULL, &minLoc, NULL, Mat() );
    matchLoc = minLoc;
    
    minVal = abs(minVal); //am i losing important info here?
    
    if(!relaxed && (minVal > thresh)){//returns 0,null if threshold violated
        return new pair<double,Point>;
    }
    
    pair<double,Point>* returnPair = new pair<double,Point>;
    returnPair->first = minVal;
    returnPair->second = matchLoc;
    
    return returnPair;
}

/*
 This function is only necessary if different input videos have the game window at different sizes. I have no idea if this happens. If we find later that it does, we can use this function to figure out what to scale everything by (only relevant to reading the HUD, since characters are going to change scale regardless of window size)
*/

void getScale(cv::Mat scene)
{
    Mat scaleR = imread(cwd.string() + "/Resources/scaleR.png",1), resizedScaleR;
	if (scaleR.cols == 0) {
		cout << "Error reading" << endl;
	}
    Size size = scaleR.size();
    double scaleMatch,bestScaleMatch = 100;
    float bestScale = 0;
    pair<double,Point>* matchPoint;
    for(float i=.5 ; i <= 2 ; i = i+.05)
    {
        resize(scaleR, resizedScaleR, cvSize(size.width * i, size.height * i));
        matchPoint = templateMatch(scene,resizedScaleR,true);
        scaleMatch = matchPoint->first;
        if((scaleMatch < bestScaleMatch) && scaleMatch > 0)
        {
            bestScaleMatch = scaleMatch;
            bestScale = i;
        }
    }
    
    videoScale = bestScale;
}



//this just doesn't work//
int getCurrentTime(cv::Mat scene) //good lord this is awful
{
    Mat garb_scene;
    scene.copyTo(garb_scene);
    
    Mat num;
    
    pair<double,Point>* matchPoint;
    
    list<Point> numbers;
    int littleNums = 0;
    int yLoc = 0;
    
    for(int i=0 ; i <= 9 ; i++)
    {
        switch (i) {
            case 0:
                num = Zero;
                break;
            case 1:
                num = One;
                break;
            case 2:
                num = Two;
                break;
            case 3:
                num = Three;
                break;
            case 4:
                num = Four;
                break;
            case 5:
                num = Five;
                break;
            case 6:
                num = Six;
                break;
            case 7:
                num = Seven;
                break;
            case 8:
                num = Eight;
                break;
            case 9:
                num = Nine;
                break;
        }
        
        Size size = num.size();
        if(littleNums<2)
        {
            matchPoint = templateMatch(garb_scene, num, false, .22);
            if(matchPoint->first > 0){
				cout << "Matched " << i << endl;
                rectangle(garb_scene, matchPoint->second, Point( matchPoint->second.x + num.cols , matchPoint->second.y + num.rows ), Scalar(255, 0, 0), 2, 8, 0 );
                imshow("little " + to_string(i) + " " + to_string(matchPoint->first),garb_scene);
                //waitKey(0);
                numbers.push_back(*new Point(i,matchPoint->second.x));
                if(numbers.size() == 6){
                    i = 10;
                    continue;
                }
                littleNums++;
                i = -1;
                continue;
            }
        }
        resize(num, num, cvSize(size.height * 1.3333, size.width * 1.3333));
        matchPoint = templateMatch(garb_scene, num, false, .45);
        if(matchPoint->first > 0){
            if(yLoc > 0 && (abs(yLoc - matchPoint->second.y)>50)){
                continue;
            }
			cout << "Matched large " << i << endl;
            rectangle(garb_scene, matchPoint->second, Point( matchPoint->second.x + num.cols , matchPoint->second.y + num.rows ), Scalar(255, 255, 0), 2, 8, 0 );
            //imshow("big " + to_string(i) + " " + to_string(matchPoint->first),garb_scene);
            /* waitKey(0); */
            if(yLoc == 0){
                yLoc = matchPoint->second.y;
            }
            numbers.push_back(*new Point(i,matchPoint->second.x));
            if(numbers.size() == 6){
                i = 10;
                continue;
            }
            i = -1;
            continue;
        }
    }
    
    /* numbers.sort(compPoints); */
    
    /* list<Point>::iterator iter; */
    int milliseconds = 0;
    /* int position = 0; */
    /* for (iter = numbers.begin() ; iter != numbers.end(); ++iter){ */
    /*     switch(position){ */
    /*         case 0: */
    /*             milliseconds += iter->x * 600000; */
    /*             position++; */
    /*             break; */
    /*         case 1: */
    /*             milliseconds += iter->x * 60000; */
    /*             position++; */
    /*             break; */
    /*         case 2: */
    /*             milliseconds += iter->x * 10000; */
    /*             position++; */
    /*             break; */
    /*         case 3: */
    /*             milliseconds += iter->x * 1000; */
    /*             position++; */
    /*             break; */
    /*         case 4: */
    /*             milliseconds += iter->x * 100; */
    /*             position++; */
    /*             break; */
    /*         case 5: */
    /*             milliseconds += iter->x * 10; */
    /*             position++; */
    /*             break; */
    /*     } */
    /* } */

    return 480000-milliseconds;
}

bool compPoints (Point i,Point j) { return (i.y<j.y); }

void setUpNumbers(cv::Mat scene)
{
    getScale(scene);
    
    cout << videoScale << endl; //why is this sometimes broken
    
    Zero = imread(cwd.string() + "/Resources/zero.png",1);
    Size size = Zero.size();
    resize(Zero,Zero,cvSize(size.width * videoScale, size.height * videoScale));
    
    One = imread(cwd.string() + "/Resources/one.png",1);
    size = One.size();
    resize(One,One,cvSize(size.width * videoScale, size.height * videoScale));

    Two = imread(cwd.string() + "/Resources/two.png",1);
    size = Two.size();
    resize(Two,Two,cvSize(size.width * videoScale, size.height * videoScale));
    
    Three = imread(cwd.string() + "/Resources/three.png",1);
    size = Three.size();
    resize(Three,Three,cvSize(size.width * videoScale, size.height * videoScale));
    
    Four = imread(cwd.string() + "/Resources/four.png",1);
    size = Four.size();
    resize(Four,Four,cvSize(size.width * videoScale, size.height * videoScale));
    
    Five = imread(cwd.string() + "/Resources/five.png",1);
    size = Five.size();
    resize(Five,Five,cvSize(size.width * videoScale, size.height * videoScale));
    
    Six = imread(cwd.string() + "/Resources/six.png",1);
    size = Six.size();
    resize(Six,Six,cvSize(size.width * videoScale, size.height * videoScale));
    
    Seven = imread(cwd.string() + "/Resources/seven.png",1);
    size = Seven.size();
    resize(Seven,Seven,cvSize(size.width * videoScale, size.height * videoScale));
    
    Eight = imread(cwd.string() + "/Resources/eight.png",1);
    size = Eight.size();
    resize(Eight,Eight,cvSize(size.width * videoScale, size.height * videoScale));
    
    Nine = imread(cwd.string() + "/Resources/nine.png",1);
    size = Nine.size();
    resize(Nine,Nine,cvSize(size.width * videoScale, size.height * videoScale));
}

//fuck this: http://supersmashbros.wikia.com/wiki/Stock_Glitch
pair<string, string> getMatchup(Mat scene){
    using namespace boost::filesystem;
    path dir_path = cwd.string() + "/Resources/icons";
    
    directory_iterator it(dir_path), eod;
    
    Mat icon;
    CvSize size;
    pair<double, Point> matchPoint;
    double bestMatchVal=999999,otherBestMatchVal=999999;
    string bestMatchString,otherBestMatchString;
    
    BOOST_FOREACH(boost::filesystem::path const &p, std::make_pair(it, eod))
    {
        if(is_regular_file(p))
        {
            icon = cv::imread(p.string());
            size = icon.size();
            if(size.width == 0){//skip rando hidden files
                continue;
            }
            resize(icon,icon,cvSize(size.width * videoScale, size.height * videoScale));
            matchPoint = *templateMatch(scene, cv::imread(p.string()));
            
            if((matchPoint.first > 0) && (matchPoint.first < otherBestMatchVal))
            {
                if(matchPoint.first < bestMatchVal)
                {
                    bestMatchVal = matchPoint.first;
                    bestMatchString = p.string();
                }
                else
                {
                    otherBestMatchVal = matchPoint.first;
                    otherBestMatchString = p.string();
                }
            }
        }
    }
    return pair<string, string>(bestMatchString.substr(bestMatchString.find_last_of("/")+1), otherBestMatchString.substr(otherBestMatchString.find_last_of("/")+1));
}

/*
********ALSO FOR KEYPOINT MATCHING. MAY NEED TO FALL BACK ON THIS SOMEDAY, SO LEAVING IN********
//still working out what this can do
//some dude on the interweb said it can help with descriptor matching
void cornerHarrisDemo( int, char* filename, void* )
{
    using namespace cv;
    /// Global variables
    Mat src_gray;
    int thresh = 200;
    
    src_gray = cv::imread( filename, 0 );
    
    String source_window = "Source image";
    String corners_window = "Corners detected";
    
    Mat dst, dst_norm, dst_norm_scaled;
    dst = Mat::zeros( src_gray.size(), CV_32FC1 );
    
    /// Detector parameters
    int blockSize = 2;
    int apertureSize = 5;
    double k = 0.04;
    
    /// Detecting corners
    cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
    
    /// Normalizing
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );
    
    /// Drawing a circle around corners
    for( int j = 0; j < dst_norm.rows ; j++ )
    { for( int i = 0; i < dst_norm.cols; i++ )
    {
        if( (int) dst_norm.at<float>(j,i) > thresh )
        {
            circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
        }
    }
    }
    /// Showing the result
    namedWindow( corners_window, CV_WINDOW_AUTOSIZE );
    imshow( corners_window, dst_norm_scaled );
    waitKey(0);
    using namespace std;
}
 */
