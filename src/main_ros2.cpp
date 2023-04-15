#include "opencv2/calib3d.hpp"
#include <iostream>
#include <array>
#include <fstream>
#include <string>
#include "ImageSubscriberROS2.h"

using namespace cv;
using namespace std;

const cv::Matx33d gazeboMatrix = {
    0,0,1,
    -1,0,0,
    0,-1,0
};

bool isRotationMatrix(cv::Matx33d R)
{
    cv::Matx33d Rt;
    cv::transpose(R, Rt);
    cv::Matx33d shouldBeIdentity = Rt * R;
    cv::Matx33d I = I.eye();

    return  norm(I, shouldBeIdentity) < 1e-6;
}

cv::Matx31d rotationMatrixToEulerAngles(cv::Matx33d R)
{
    float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );
    bool singular = sy < 1e-6; // If
    cv::Matx31d temp;
    if (!singular)
    {
        temp(0) = atan2(R(2,1) , R(2,2));
        temp(1) = atan2(-R(2,0), sy);
        temp(2) = atan2(R(1,0), R(0,0));
    }
    else
    {
        temp(0) = atan2(-R(1,2), R(1,1));
        temp(1) = atan2(-R(2,0), sy);
        temp(2) = 0;
    }
    return temp;
}

vector<Point2f> CornersFromClicks;
void CallBackFunc(int event, int x, int y, int flags, void* imgptr)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        Mat & img = (*(Mat*)imgptr);
        Point pt1 = Point(x, y);
        circle(img, pt1, 1, Scalar(255, 0, 0), 1, 2, 0);
        imshow("clickCorners", img);
        waitKey(1);
        Point2f temp;
        temp.x = x;
        temp.y = y;
        if(x!=0){CornersFromClicks.push_back(temp); cout << CornersFromClicks.size() << endl;}
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " <<  y << ")" << endl;
    }
}

bool clickTodetectChessboardCorner(Mat clickimg, Size patternsize)
{
    namedWindow("clickCorners",cv::WINDOW_NORMAL);
    resizeWindow("clickCorners", clickimg.cols, clickimg.rows);
    setMouseCallback("clickCorners", CallBackFunc, &clickimg);
    imshow("clickCorners", clickimg);
    waitKey(0);
    return true;
}

void getWorldToVoxelMat(std::array<std::vector<std::vector<cv::Point3f> >, 3> objpoints, int camera_id)
{
    printf("------------------- get world to voxel matrix -------------------\n");
    //get rotation,translation matrix from world to voxel using estimateaffine3d for 1 camera
    cv::Mat world2voxel;
    std::vector<uchar> inliers;
    std::vector<cv::Point3f> world_points;
    std::vector<cv::Point3f> voxel_points;

    double voxel_center_x = 256.0, voxel_center_y = 256.0, voxel_center_z = 100.0;  // lower z limit is 77
    

    // world_points.push_back(cv::Point3d(0,0,0)); voxel_points.push_back(cv::Point3d(256,256,100));
    // world_points.push_back(cv::Point3d(230,0,0)); voxel_points.push_back(cv::Point3d(256+23,256,100));
    // world_points.push_back(cv::Point3d(230,460,0)); voxel_points.push_back(cv::Point3d(256+23,256+46,100));
    // world_points.push_back(cv::Point3d(0,460,0)); voxel_points.push_back(cv::Point3d(256,256+46,100));

    


    if(camera_id==0 || camera_id==1){
        for(int i{0}; i<2; i++){
            for(int j{0}; j<3; j++){
                world_points.push_back(cv::Point3f(objpoints[0][0][i*6*2+j*2].x, objpoints[0][0][i*6+j*2].y, 0));
                voxel_points.push_back(cv::Point3f(voxel_center_x+(objpoints[0][0][i*6*2+j*2].x)*0.1, voxel_center_y+(objpoints[0][0][i*6+j*2].y)*0.1, voxel_center_z));
                printf("worldpPoints[0][0][%d]  x = %f, y= %f\n", i*6*2+j*2, objpoints[0][0][i*6*2+j*2].x, objpoints[0][0][i*6*2+j*2].y);
                printf("voxelPoints[0][0][%d]  x = %f, y= %f\n", i*6*2+j*2, voxel_center_x+(objpoints[0][0][i*6*2+j*2].x)*0.1, voxel_center_y+(objpoints[0][0][i*6+j*2].y)*0.1);
            }
        }
    }
    else if(camera_id==2) {
        for(int i{1}; i>=0; i--){
            for(int j{2}; j>=0; j--){
                world_points.push_back(cv::Point3f(objpoints[camera_id][0][i*6*2+j*2].x, objpoints[camera_id][0][i*6+j*2].y, 0));
                voxel_points.push_back(cv::Point3f(voxel_center_x+(objpoints[camera_id][0][i*6*2+j*2].x)*0.1, voxel_center_y+(objpoints[camera_id][0][i*6+j*2].y)*0.1, voxel_center_z));
                printf("worldpPoints[%d][0][%d]  x = %f, y= %f\n", camera_id, i*6*2+j*2, objpoints[camera_id][0][i*6*2+j*2].x, objpoints[camera_id][0][i*6*2+j*2].y);
                printf("voxelPoints[%d][0][%d]  x = %f, y= %f\n", camera_id, i*6*2+j*2, voxel_center_x+(objpoints[camera_id][0][i*6*2+j*2].x)*0.1, voxel_center_y+(objpoints[camera_id][0][i*6+j*2].y)*0.1);
            }
        }
    }
    else {printf("camera_id is wrong\n"); return;}


    int ret = cv::estimateAffine3D(world_points, voxel_points, world2voxel,inliers);
    if(ret!=0) {cout << "world to voxel :" << world2voxel << endl;}
    else {cout << "world to voxel failed" << endl;}
    printf("------------------- end -------------------\n");
}



// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6,4}; 

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string cam0_topic = "/azure_kinect/master/rgb/";
    std::string cam1_topic = "/azure_kinect/sub1/rgb/";
    std::string cam2_topic = "/azure_kinect/sub2/rgb/";
    auto cam0_node = std::make_shared<ImageSubROS2>(argc, argv, 0, cam0_topic);
    auto cam1_node = std::make_shared<ImageSubROS2>(argc, argv, 1, cam1_topic);
    auto cam2_node = std::make_shared<ImageSubROS2>(argc, argv, 2, cam2_topic);

    

    while (rclcpp::ok())
    {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(cam0_node);
        executor.add_node(cam1_node);
        executor.add_node(cam2_node);
        executor.spin();
        rclcpp::shutdown();
    }

    // Creating vector to store vectors of 3D points for each checkerboard image
    std::array<std::vector<std::vector<cv::Point3f> >, 3> objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::array<std::vector<std::vector<cv::Point2f> > , 3> imgpoints;
    std::array<cv::Matx33d, 3> cameraMatrix;
  
                           //fx             cx         fy     cy 
    /*double camMatemp[3][9] = {{973.055, 0, 1025.89, 0, 972.716, 779.694, 0, 0, 1}, 
                            {971.502, 0, 1027.51, 0, 971.037, 784.374, 0, 0, 1},
                            {974.333, 0, 1019.83, 0, 973.879, 782.927, 0, 0, 1}};*/

    /*double camMatemp[3][9] = {{master.fx, 0, master.cx, 0, master.fy, master.cy, 0, 0, 1}, 
                            {sub1.fx, 0, sub1.cx, 0, sub1.fy, sub1.cy, 0, 0, 1},
                            {sub2.fx, 0, sub2.cx, 0, sub2.fy, sub2.cy, 0, 0, 1}};*/
    /* double camMatemp[3][9] = {{971.502,0, 971.037,0, 1027.51, 784.374, 0,0,1}, 
                            {973.055,0, 972.716,0, 1025.89, 779.694 , 0,0,1},
                            {974.243,0, 973.095, 0,1021.18, 771.734, 0,0,1}};        */      
    // 1536P
    // double camMatemp[3][9] = {{974.333,0, 1019.83,0,973.879, 782.927, 0,0,1}, 
                            // {973.055,0, 1025.89,0,972.716, 779.694 , 0,0,1},
                            // {974.243,0, 1021.18, 0, 974.095, 771.734, 0,0,1}};
    // 720P                                          
    double camMatemp[3][9] = {{608.958,0, 637.205,0,608.674, 369.142, 0,0,1}, 
                            {608.16,0, 640.996,0,607.948, 367.121 , 0,0,1},
                            {608.902,0, 638.053, 0, 608.809, 362.146, 0,0,1}};   

    for(int r=0; r<3; r++)
    {
        cameraMatrix[r] = cv::Mat(3, 3, CV_64FC1, camMatemp[r]);
        std::cout << "iintrinsic input" << std::endl;
        std::cout << cameraMatrix[r] << std::endl;
    }
    
    


    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;
    // Path of the folder containing checkerboard images
    std::string path = "/home/do/ros2_ws/src/camera_calibration_azure/images/*.png";

    cv::glob(path, images);

    cv::Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_pts;
    bool success;
    int camid = 0;
    // Looping over all the images in the directory
    for(int i{0}; i<images.size(); i++)
    {
        cout << images.size() <<endl;
        frame = cv::imread(images[i]);
        std::size_t found = images[i].find("2");
        if (found!=std::string::npos) camid= 2; 
        found = images[i].find("1");
        if (found!=std::string::npos) camid= 1; 
        found = images[i].find("0");
        if (found!=std::string::npos) camid= 0; 
        cout <<"camid" << camid <<endl;
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

        // Defining the world coordinates for 3D points
        std::vector<cv::Point3f> objp;
        if(camid==0 || camid==1){
            for(int i{0}; i<CHECKERBOARD[1]; i++) { //4
                for(int j{0}; j<CHECKERBOARD[0]; j++) //6
                    objp.push_back(cv::Point3f(i * 115,j* 115,0));
            }
        }
        else if(camid==2){
            for(int i{CHECKERBOARD[1]-1}; i>=0; i--) { //4
                for(int j{CHECKERBOARD[0]-1}; j>=0; j--) //6
                    objp.push_back(cv::Point3f(i * 115,j* 115,0));
            }
        }
        else printf("camid error");

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        //cout << "success" << success <<endl;
        /* 
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display 
         * them on the images of checker board
        */
        if(!success) 
        {
            success = clickTodetectChessboardCorner(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]));
            for(int k=0; k<CornersFromClicks.size();k++) {corner_pts.push_back(CornersFromClicks[k]);}
        }
        // cout << "clickedcornersNum" << CornersFromClicks.size() << endl;
        //    cout << "NumofCorners" << corner_pts.size() << endl;
        // cout << "success" << success <<endl;
        if(success)
        {
            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 50, 0.001);
      
            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray,corner_pts,cv::Size(5,5), cv::Size(-1,-1),criteria);
      
            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
        
            objpoints[camid].push_back(objp);
            imgpoints[camid].push_back(corner_pts);
        
            // cout << "objpoints if success" <<  objpoints[camid].size() << endl;
            //  cout << "NumofCorners_if success" << corner_pts.size() << endl;
      
            CornersFromClicks.clear();
        }
        
        

        cv::imshow("Image",frame);
        cv::waitKey(0);

        getWorldToVoxelMat(objpoints,camid);
    }

    cv::destroyAllWindows(); 
    // print all points of objpoints and imgpoints
    // for(int camnum{0};camnum<3;camnum++){
    //     cout<< "now the cam"<< camnum <<endl;
    //     for(int i{0};i<4;i++){
    //         for(int j{0}; j<6; j++)
    //         {
    //             printf("objpoints[%d][0][%d]  x = %f, y= %f\n", camnum, i*6+j, objpoints[camnum][0][i*6+j].x,objpoints[camnum][0][i*6+j].y);
    //             printf("imgpoints[%d][0][%d]  x = %f, y= %f\n", camnum, i*6+j, imgpoints[camnum][0][i*6+j].x,imgpoints[camnum][0][i*6+j].y);
    //         }
    //     }
    // }

    
    

    

    cv::Mat R[3],T[3];
    cv::Matx31d outputT;
    cv::Matx41d distCoeffs;
    cv::Matx33d rvec[3]; 
    cv::Matx31d rveceuler[3];  

    distCoeffs = distCoeffs.zeros();
    std::ofstream opnefile("/home/do/ros2_ws/src/camera_calibration_azure/ExtrinsicFile.txt");

    for(int i=0; i<3; i++)
    { double meanError =100;
    //std::vector<cv::Point2f> projectedPoints;
    
    //cv::calibrateCamera(objpoints[i], imgpoints[i], cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T);//에러 왜 큰지부터.. 
    std::cout << cameraMatrix[i] << std::endl;
    cv::solvePnP(objpoints[i][0], imgpoints[i][0], cameraMatrix[i], distCoeffs, R[i], T[i]);
 //For GPU-Voxel
    cv::Rodrigues(R[i],rvec[i]);
    assert(isRotationMatrix(rvec[i]));
    rvec[i] = rvec[i];//* gazeboMatrix.inv();
    rveceuler[i]= rotationMatrixToEulerAngles(rvec[i]);
    outputT = T[i];
    /*
    if(i>0)
    {                          
    cv::Rodrigues(R[i],rvec[i]);
    assert(isRotationMatrix(rvec[i]));
    rvec[i] = rvec[i]* gazeboMatrix.inv();
    rveceuler[i]= rotationMatrixToEulerAngles(rvec[i]);
    outputT = T[i];
    }
    else //여기가 0일때 starting코드.
    {                          
    cv::Rodrigues(R[i],rvec[i]);
    assert(isRotationMatrix(rvec[i]));
    //cv::Matx33d rvec0 = rvec[0]*rvec[i].inv();// gazeboMatrix.inv();
    cv::Matx33d rvec0 = rvec[i].inv();
    //rvec0 = rvec[0] * rvec0;
    rveceuler[i]= rotationMatrixToEulerAngles(rvec0);
    cv::Matx31d negT((double*)T[i].ptr());
    cv::Matx31d Tx((double*)T[0].ptr());
  
    outputT = (-1* rvec[i].inv()) * negT;
    //outputT = (rvec[0] * outputT) + Tx;
    }
    */
    //while(meanError >1.51)
    {
    double totalError =0;
    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(objpoints[i][0], R[i], T[i], cameraMatrix[i], distCoeffs, projectedPoints); 
    double error = cv::norm(imgpoints[i][0], projectedPoints, cv::NORM_L2);
    meanError = error/objpoints[i][0].size();
    }
    
    
    std::cout << " "<< std::endl;
    std::cout << "camera ID : " << i << std::endl;
    std::cout << "meanError"<<meanError << std::endl;
    std::cout << "cameraMatrix : " << cameraMatrix[i] << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;
    std::cout << "Rotation vector Rodriguez : " << R[i] << std::endl;
    std::cout << "Rotation Matrix : " << rvec[i] << std::endl;
    std::cout << "Rotation Matrix Inv: " << rvec[i].inv() << std::endl;
    std::cout << "Rotation vector euler : " << rveceuler[i] << std::endl;
    std::cout << "Translation vector : " << T[i] << std::endl;
    std::cout << "Translation vector Rotation Inversed: " << outputT << std::endl;
    if (opnefile.is_open()) {
        opnefile << outputT(0)/1000 << " "<< outputT(1)/1000 <<" " <<outputT(2)/1000 <<" "<<rveceuler[i](0) << " "<<rveceuler[i](1)<<" "<<rveceuler[i](2) <<std::endl;
        
    }
    

    //cv::Matx33d m33((double*)T.ptr());
    //std::cout << "Translation vector Gazebo Inv : " << (m33 * gazeboMatrix.inv()) << std::endl;
    }
    opnefile.close();
    return 0;
}
