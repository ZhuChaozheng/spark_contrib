/*
 * Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
 * Released to public domain under terms of the BSD Simplified license.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the organization nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *   See <http://www.opensource.org/licenses/bsd-license>
 */

 #include "opencv2/core.hpp"
 #include "opencv2/face.hpp"
 #include "opencv2/highgui.hpp"
 #include "opencv2/imgproc.hpp"
 #include "opencv2/objdetect.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/String.h>

using namespace cv;
using namespace cv::face;
using namespace std;

static const string OPENCV_WINDOW = "face_recognizer";

class FaceRecogition {
  ros::NodeHandle nh_;
  ros::Publisher string_pub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  // global variable
  CascadeClassifier haar_cascade;
  Ptr<EigenFaceRecognizer> model;

  int im_width;
  int im_height;

  // pixel_distance invariable
  int pixel_distance_x;
  int pixel_distance_y;

  string fn_haar, fn_csv;


public:
  FaceRecogition() : it_(nh_) {

    // preload train and load face model
    trainImage();
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &FaceRecogition::imageCb, this);
    cout << "the out is " << image_sub_ << endl;
    image_pub_ = it_.advertise("/facerecogition/output_video", 1);
    // publish string message to topic "/pixel_distance/steering"
    string_pub_ = nh_.advertise<std_msgs::String>("/pixel_distance/steering", 1);

    cv::namedWindow(OPENCV_WINDOW);

}

~FaceRecogition() {
  cv::destroyWindow(OPENCV_WINDOW);
}

void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';') {
    ifstream file(filename.c_str(), ifstream::in);
    if (!file) {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, path, classlabel;
    while (getline(file, line)) {
        stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty()) {
            images.push_back(imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        }
    }

}

void trainImage() {

  // grab the parameters
  nh_.param("/face_recognizer/haarcascade_frontface", fn_haar, std::string("data/haar_face.xml"));
  nh_.param("/face_recognizer/csv", fn_csv, std::string("data/csv.yaml"));
  cout << fn_csv << endl;
  // Get the path to your CSV:
  // string fn_haar = "data/haarcascades/haarcascade_frontalface_alt.xml";
  // string fn_csv = "data/csv.yaml";

  // These vectors hold the images and corresponding labels:
  vector<Mat> images;
  vector<int> labels;
  // Read in the data (fails if no valid input filename is given, but you'll get an error message):
  try {
      read_csv(fn_csv, images, labels);
  } catch (cv::Exception& e) {
      cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
      // nothing more we can do
      exit(1);
  }

  // Get the height from the first image. We'll need this
  // later in code to reshape the images to their original
  // size AND we need to reshape incoming faces to this size:
  im_width = images[0].cols;
  cout << im_width << endl;
  im_height = images[0].rows;
  // Create a FaceRecognizer and train it on the given images:
  // Attention please: if you reference the official opencv docs
  // you may discover a mistake that there is a grammar errors in openv3
  // in details on the blew content:
  // Ptr<FaceRecognizer> model = createFisherFaceRecognizer();
  model = EigenFaceRecognizer::create();
  model->train(images, labels);
  // That's it for learning the Face Recognition model. You now
  // need to create the classifier for the task of Face Detection.
  // We are going to use the haar cascade you  have specified in the
  // command line arguments:
  //

  haar_cascade.load(fn_haar);
  cout << "train and load model over!" << endl;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg) {
  ROS_INFO("Processing the Image to locate the Object....");

  cv_bridge::CvImagePtr cv_ptr_;
  try {
    cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Holds the current frame from ros topics:
  Mat &original = cv_ptr_->image;

  // Convert the current frame to grayscale:
  Mat gray;
  cvtColor(original, gray, CV_BGR2GRAY);


  // Find the faces in the frame:
  vector< Rect_<int> > faces;
  haar_cascade.detectMultiScale(gray, faces);
  // At this point you have the position of the faces in
  // faces. Now we'll get the faces, make a prediction and
  // annotate it in the video. Cool or what?
  for(int i = 0; i < faces.size(); i++) {
      // Process face by face:
      Rect face_i = faces[i];

      // Crop the face from the image. So simple with OpenCV C++:
      Mat face = gray(face_i);
      // Resizing the face is necessary for Eigenfaces and Fisherfaces. You can easily
      // verify this, by reading through the face recognition tutorial coming with OpenCV.
      // Resizing IS NOT NEEDED for Local Binary Patterns Histograms, so preparing the
      // input data really depends on the algorithm used.
      //
      // I strongly encourage you to play around with the algorithms. See which work best
      // in your scenario, LBPH should always be a contender for robust face recognition.
      //
      // Since I am showing the Fisherfaces algorithm here, I also show how to resize the
      // face you have just found:
      Mat face_resized;
      resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
      // Now perform the prediction, see how easy that is:
      int prediction = model->predict(face_resized);
      // And finally write all we've found out to the original image!
      // First of all draw a green rectangle around the detected face:
      rectangle(original, face_i, CV_RGB(0, 255,0), 1);
      // Create the text we will annotate the box with:
      string box_text = format("Prediction = %d", prediction);
      // Calculate the position for annotated text (make sure we don't
      // put illegal values in there):
      int pos_x = std::max(face_i.tl().x - 10, 0);
      int pos_y = std::max(face_i.tl().y - 10, 0);
      // And now put it into the image:
      putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);

      pixel_distance_x = face_i.x + face_i.width/2 - original.cols/2;
      pixel_distance_y = face_i.y + face_i.height/2 - original.rows/2;

      std_msgs::String message;
      std::stringstream ss;
      ss << pixel_distance_x << "," << pixel_distance_y;
      message.data = ss.str();
      ROS_INFO("%s", message.data.c_str());
      string_pub_.publish(message);

  }
  // Show the result:
  imshow(OPENCV_WINDOW, cv_ptr_->image);
  cv::waitKey(1);

  // Output modified  video stream
  image_pub_.publish(cv_ptr_->toImageMsg());

}

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "face_recognizer");
  FaceRecogition fr;
  // ros::NodeHandle nh;
  // ros::Publisher pub = nh.advertise<std_msgs::String>("/pixel_distance", 5);
  // std_msgs::String str;
  // str.data = "hello world";
  // pub.publish(str);
  ros::spin();
  return 0;
}
