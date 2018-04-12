/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include <mav_msgs/common.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>


#include <ros/ros.h>
#include <find_object_2d/ObjectsStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <QTransform>
#include <QColor>

image_transport::Publisher imagePub;
ros::Publisher pose_detection_pub;
Eigen::Vector3d L_p_LB_;                // Mav position in local coordinates.

/**
 * IMPORTANT :
 *      Parameter General/MirrorView must be false
 *      Parameter Homography/homographyComputed must be true
 */
void OdometryCallback(const nav_msgs::Odometry& msg)
{
  	Eigen::Affine3d T_LB;
  	tf::poseMsgToEigen(msg.pose.pose, T_LB);
  	L_p_LB_ = T_LB.translation();
}
void objectsDetectedCallback(
		const std_msgs::Float32MultiArrayConstPtr & msg)
{
	geometry_msgs::PoseStamped L_pose_detected_;
  	geometry_msgs::PoseArray tag_pose_array;

	printf("---\n");
	const std::vector<float> & data = msg->data;
	if(data.size())
	{
		for(unsigned int i=0; i<data.size(); i+=12)
		{
			// get data
			int id = (int)data[i];
			float objectWidth = data[i+1];
			float objectHeight = data[i+2];

			// Find corners Qt
			QTransform qtHomography(data[i+3], data[i+4], data[i+5],
									data[i+6], data[i+7], data[i+8],
									data[i+9], data[i+10], data[i+11]);

			QPointF qtTopLeft = qtHomography.map(QPointF(0,0));
			QPointF qtTopRight = qtHomography.map(QPointF(objectWidth,0));
			QPointF qtBottomLeft = qtHomography.map(QPointF(0,objectHeight));
			QPointF qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight));

			printf("Object %d detected, Qt corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",
					id,
					qtTopLeft.x(), qtTopLeft.y(),
					qtTopRight.x(), qtTopRight.y(),
					qtBottomLeft.x(), qtBottomLeft.y(),
					qtBottomRight.x(), qtBottomRight.y());
			
			//float area = (qtTopLeft - qtTopRight).x() * (qtTopLeft - qtBottomLeft).y();
			//printf("area1: %f", area);

			float x0 = qtTopLeft.x();
			float x1 = qtTopRight.x();
			float x2 = qtBottomRight.x();
			float x3 = qtBottomLeft.x();
			float y0 = qtTopLeft.y();
			float y1 = qtTopRight.x();
			float y2 = qtBottomRight.x();
			float y3 = qtBottomLeft.x();
			float x[4] = {x0,x1,x2,x3};
			float y[4] = {y0,y1,y2,y3};
		    float area = fabs(x[0]*y[1]-x[1]*y[0]+x[1]*y[2]-x[2]*y[1]+x[2]*y[3]-x[3]*y[2]+x[3]*y[0]-x[0]*y[3])/2;
			printf("area: %f", area);
			
			float frameWidth = 1024;
			float frameHeight = 768;
			float magnifyFactor = 1.5;
			float heightFactor = sqrt(4000)*2.5;
			L_pose_detected_.pose.position.x = ((qtTopLeft + qtTopRight+ qtBottomLeft + qtBottomRight).x()/4/frameWidth-0.5)*L_p_LB_.z()*magnifyFactor;
			L_pose_detected_.pose.position.y = ((qtTopLeft + qtTopRight+ qtBottomLeft + qtBottomRight).y()/4/frameHeight-0.5)*L_p_LB_.z()*magnifyFactor;
			L_pose_detected_.pose.position.z = sqrt(1 / area) * heightFactor;
			//L_pose_detected_.pose.position.z = 2.5;
			double idle_yaw_ = atan2(( qtTopLeft.y() - qtTopRight.y() ), ( qtTopLeft.x() - qtTopRight.x() ) );
  			Eigen::Quaterniond idle_orien_ = mav_msgs::quaternionFromYaw(idle_yaw_);
  			Eigen::Quaterniond idle_orien;
  			tf::quaternionEigenToMsg(idle_orien_,L_pose_detected_.pose.orientation);



    		tag_pose_array.poses.push_back(L_pose_detected_.pose);



		}
	}
	else
	{
		printf("No objects detected.\n");
	}
	pose_detection_pub.publish(tag_pose_array);
}
void imageObjectsDetectedCallback(
		const sensor_msgs::ImageConstPtr & imageMsg,
		const find_object_2d::ObjectsStampedConstPtr & objectsMsg)
{
	if(imagePub.getNumSubscribers() > 0)
	{
		const std::vector<float> & data = objectsMsg->objects.data;
		if(data.size())
		{
			for(unsigned int i=0; i<data.size(); i+=12)
			{
				// get data
				int id = (int)data[i];
				float objectWidth = data[i+1];
				float objectHeight = data[i+2];

				// Find corners OpenCV
				cv::Mat cvHomography(3, 3, CV_32F);
				cvHomography.at<float>(0,0) = data[i+3];
				cvHomography.at<float>(1,0) = data[i+4];
				cvHomography.at<float>(2,0) = data[i+5];
				cvHomography.at<float>(0,1) = data[i+6];
				cvHomography.at<float>(1,1) = data[i+7];
				cvHomography.at<float>(2,1) = data[i+8];
				cvHomography.at<float>(0,2) = data[i+9];
				cvHomography.at<float>(1,2) = data[i+10];
				cvHomography.at<float>(2,2) = data[i+11];
				std::vector<cv::Point2f> inPts, outPts;
				inPts.push_back(cv::Point2f(0,0));
				inPts.push_back(cv::Point2f(objectWidth,0));
				inPts.push_back(cv::Point2f(objectWidth,objectHeight));
				inPts.push_back(cv::Point2f(0,objectHeight));
				inPts.push_back(cv::Point2f(objectWidth/2,objectHeight/2));
				cv::perspectiveTransform(inPts, outPts, cvHomography);

				cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageMsg);

				cv_bridge::CvImage img;
				img = *imageDepthPtr;
				std::vector<cv::Point2i> outPtsInt;
				outPtsInt.push_back(outPts[0]);
				outPtsInt.push_back(outPts[1]);
				outPtsInt.push_back(outPts[2]);
				outPtsInt.push_back(outPts[3]);
				QColor color(QColor((Qt::GlobalColor)((id % 10 + 7)==Qt::yellow?Qt::darkYellow:(id % 10 + 7))));
				cv::Scalar cvColor(color.red(), color.green(), color.blue());
				cv::polylines(img.image, outPtsInt, true, cvColor, 3);
				cv::Point2i center = outPts[4];
				cv::putText(img.image, QString("(%1, %2)").arg(center.x).arg(center.y).toStdString(), center, cv::FONT_HERSHEY_SIMPLEX, 0.6, cvColor, 2);
				cv::circle(img.image, center, 1, cvColor, 3);
				imagePub.publish(img.toImageMsg());
			}
		}
	}
}

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, find_object_2d::ObjectsStamped> MyExactSyncPolicy;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "objects_detected");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Simple subscriber
    ros::Subscriber sub;
    sub = nh.subscribe("objects", 1, objectsDetectedCallback);
	ros::Subscriber odometry_sub = nh.subscribe("/jay/rovio/odometry", 1, OdometryCallback);

    // Synchronized image + objects example
    image_transport::SubscriberFilter imageSub;
	imageSub.subscribe(it, nh.resolveName("image"), 1);
	message_filters::Subscriber<find_object_2d::ObjectsStamped> objectsSub;
	objectsSub.subscribe(nh, "objectsStamped", 1);
    message_filters::Synchronizer<MyExactSyncPolicy> exactSync(MyExactSyncPolicy(10), imageSub, objectsSub);
    exactSync.registerCallback(boost::bind(&imageObjectsDetectedCallback, _1, _2));

    imagePub = it.advertise("image_with_objects", 1);
	pose_detection_pub = nh.advertise<geometry_msgs::PoseArray>("Object/detection/pose",1);

    ros::spin();

    return 0;
}
