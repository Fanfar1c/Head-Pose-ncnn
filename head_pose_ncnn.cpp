#include <iostream>
#include <opencv2/opencv.hpp>
#include "FaceDetector.h"

using namespace std;

string detect() {


    float f;
    float FPS[16];
    int i, Fcnt = 0;
    cv::Mat frame;
    const int max_side = 320;

    //some timing
    chrono::steady_clock::time_point Tbegin, Tend;

    for (i = 0; i < 16; i++) FPS[i] = 0.0;

    // 3D model points.
    std::vector<cv::Point3d> model_points;
    model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));               // Nose tip
    model_points.push_back(cv::Point3d(0.0f, -300.0f, -65.0f));          // interpolate chin
    model_points.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));       // Left eye left corner
    model_points.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));        // Right eye right corner
    model_points.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
    model_points.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner

    Detector detector("C:/model/face.param", "C:/model/face.bin");


    frame = cv::imread("C:/model/9.jpg");

    if (frame.empty()) {
        cerr << "ERROR: Unable to grab from the camera" << endl;
        return "error";
    }


    float long_side = static_cast<float>((std::max)(frame.cols, frame.rows));

    float scale = max_side / long_side;
    cv::Mat img_scale;

    cv::resize(frame, img_scale, cv::Size(static_cast<int>(frame.cols * scale), static_cast<int>(frame.rows * scale)));


    std::vector<bbox> boxes;

    Tbegin = chrono::steady_clock::now();

    detector.Detect(img_scale, boxes);

    Tend = chrono::steady_clock::now();
    double yaw;


    for (size_t j = 0; j < boxes.size(); ++j) {
        f = (boxes[j].point[3]._y + boxes[j].point[4]._y) / 2; float y = 2 * f - boxes[j].point[2]._y;

        std::vector<cv::Point2d> image_points;


        image_points.push_back(cv::Point2d(static_cast<double>(boxes[j].point[2]._x) / scale, static_cast<double>(boxes[j].point[2]._y) / scale));    // Nose tip
        image_points.push_back(cv::Point2d(static_cast<double>(boxes[j].point[2]._x) / scale, static_cast<double>(y) / scale));    // Chin
        image_points.push_back(cv::Point2d(static_cast<double>(boxes[j].point[0]._x) / scale, static_cast<double>(boxes[j].point[0]._y) / scale));     // Left eye left corner
        image_points.push_back(cv::Point2d(static_cast<double>(boxes[j].point[1]._x) / scale, static_cast<double>(boxes[j].point[1]._y) / scale));    // Right eye right corner
        image_points.push_back(cv::Point2d(static_cast<double>(boxes[j].point[3]._x) / scale, static_cast<double>(boxes[j].point[3]._y) / scale));    // Left Mouth corner
        image_points.push_back(cv::Point2d(static_cast<double>(boxes[j].point[4]._x) / scale, static_cast<double>(boxes[j].point[4]._y) / scale));    // Right mouth corner



        double focal_length = frame.cols; 
        cv::Point2d center = cv::Point2d(frame.cols / 2, frame.rows / 2);
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
        cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); 

        cv::Mat rotation_vector;          
        cv::Mat translation_vector;

        // Solve for pose
        cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

        yaw = rotation_vector.at<double>(1);



        vector<cv::Point3d> nose_end_point3D;
        vector<cv::Point2d> nose_end_point2D;
        nose_end_point3D.push_back(cv::Point3d(0, 0, 300.0));
        nose_end_point3D.push_back(cv::Point3d(0, 300.0, 0.0));
        nose_end_point3D.push_back(cv::Point3d(300.0, 0.0, 0.0));
        cv::projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
        cv::line(frame, image_points[0], nose_end_point2D[1], cv::Scalar(0, 255, 0), 2);
        cv::line(frame, image_points[0], nose_end_point2D[2], cv::Scalar(0, 0, 255), 2);
        cv::line(frame, image_points[0], nose_end_point2D[0], cv::Scalar(255, 0, 0), 2);
    }


    string headOrientation;
    if (yaw < -1.0) {
        headOrientation = "Head turned to the right";
    }
    else if (yaw > 0.01) {
        headOrientation = "Head turned to the left";
    }
    else {
        headOrientation = "none";
    }

    cv::putText(frame, headOrientation, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255));

    cv::imwrite("C:/model/Result_4.jpg", frame);


    return headOrientation;



}

int video_dtect() {

    float f;
    float FPS[16];
    int i, Fcnt = 0;
    cv::Mat frame;
    const int max_side = 320;


    chrono::steady_clock::time_point Tbegin, Tend;

    for (i = 0; i < 16; i++) FPS[i] = 0.0;


    std::vector<cv::Point3d> model_points;
    model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));               // Nose tip
    model_points.push_back(cv::Point3d(0.0f, -300.0f, -65.0f));          // interpolate chin
    model_points.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));       // Left eye left corner
    model_points.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));        // Right eye right corner
    model_points.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
    model_points.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner

    Detector detector("C:/model/face.param", "C:/model/face.bin");

    cv::VideoCapture cap("C:/model/erke-esmaxan-qaida_Ue7Z5vH3.mp4");
    if (!cap.isOpened()) {
        cerr << "ERROR: Unable to open the camera" << endl;
        return 0;
    }

    cv::VideoWriter video;

    while (cap.read(frame)) {


        if (frame.empty()) {
            cerr << "ERROR: Unable to grab from the camera" << endl;
            break;
        }

        if (!video.isOpened()) {
            video.open("C:/model/outcpp.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, frame.size());
        }


        float long_side = static_cast<float>((std::max)(frame.cols, frame.rows));
        float scale = max_side / long_side;
        cv::Mat img_scale;

        cv::resize(frame, img_scale, cv::Size(static_cast<int>(frame.cols * scale), static_cast<int>(frame.rows * scale)));

        std::vector<bbox> boxes;

        Tbegin = chrono::steady_clock::now();

        detector.Detect(img_scale, boxes);

        Tend = chrono::steady_clock::now();

        // draw image
        for (size_t j = 0; j < boxes.size(); ++j) {
            f = (boxes[j].point[3]._y + boxes[j].point[4]._y) / 2; float y = 2 * f - boxes[j].point[2]._y;
            // 2D image points. we need to interpolate the position of the chin
            // in order to get the needed six points for the PnP solver
            std::vector<cv::Point2d> image_points;
            image_points.push_back(cv::Point2d(static_cast<double>(boxes[j].point[2]._x) / scale, static_cast<double>(boxes[j].point[2]._y) / scale));    // Nose tip
            image_points.push_back(cv::Point2d(static_cast<double>(boxes[j].point[2]._x) / scale, static_cast<double>(y) / scale));    // Chin
            image_points.push_back(cv::Point2d(static_cast<double>(boxes[j].point[0]._x) / scale, static_cast<double>(boxes[j].point[0]._y) / scale));     // Left eye left corner
            image_points.push_back(cv::Point2d(static_cast<double>(boxes[j].point[1]._x) / scale, static_cast<double>(boxes[j].point[1]._y) / scale));    // Right eye right corner
            image_points.push_back(cv::Point2d(static_cast<double>(boxes[j].point[3]._x) / scale, static_cast<double>(boxes[j].point[3]._y) / scale));    // Left Mouth corner
            image_points.push_back(cv::Point2d(static_cast<double>(boxes[j].point[4]._x) / scale, static_cast<double>(boxes[j].point[4]._y) / scale));    // Right mouth corner


            // Camera internals
            double focal_length = frame.cols; // Approximate focal length.
            cv::Point2d center = cv::Point2d(frame.cols / 2, frame.rows / 2);
            cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
            cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion
            // Output rotation and translation
            cv::Mat rotation_vector;          // Rotation in axis-angle form
            cv::Mat translation_vector;

            // Solve for pose
            cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

            //yaw = rotation_vector.at<double>(1);


            // Project a 3D point onto the image plane.
            // We use this to draw a line sticking out of the nose
            vector<cv::Point3d> nose_end_point3D;
            vector<cv::Point2d> nose_end_point2D;
            nose_end_point3D.push_back(cv::Point3d(0, 0, 300.0));
            nose_end_point3D.push_back(cv::Point3d(0, 300.0, 0.0));
            nose_end_point3D.push_back(cv::Point3d(300.0, 0.0, 0.0));
            cv::projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            cv::line(frame, image_points[0], nose_end_point2D[1], cv::Scalar(0, 255, 0), 2);
            cv::line(frame, image_points[0], nose_end_point2D[2], cv::Scalar(0, 0, 255), 2);
            cv::line(frame, image_points[0], nose_end_point2D[0], cv::Scalar(255, 0, 0), 2);
        }


        video.write(frame);

        cv::imshow("RPi 64 OS - 1,95 GHz - 2 Mb RAM", frame);
        char esc = cv::waitKey(50);
        if (esc == 27) break;
    }
    video.release();
    cv::destroyAllWindows();
    return 0;

}


int main(){
    string res;
    res = detect();
    video_dtect();
    
    return 0;
}

