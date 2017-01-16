#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud.h>
#include "yeti_snowplow/robot_position.h"
#include "yeti_snowplow/landmark.h"
#include "yeti_snowplow/landmark_list.h"
#include <geometry_msgs/Point.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>


#define PI 3.14159265

using namespace std;


ros::Publisher pub;//ROS publisher

yeti_snowplow::landmark_list landmarkLocationsTxt;//holds landmark locations from text file
yeti_snowplow::robot_position yeti_location;//holds previous robot location

//find the landmark locations, with respect to where Yeti was. 
yeti_snowplow::landmark_list scanLandmarks(yeti_snowplow::landmark_list landmarkLocsTXT, const sensor_msgs::PointCloud::ConstPtr& scan_pt_cloud, yeti_snowplow::robot_position prev_robot_location)
{
	//ROS_INFO_STREAM("Cloud size: " << scan_pt_cloud->points.size());
	//ROS_INFO_STREAM("Number of Landmarks: " << landmarkLocsTXT.points.size());
	yeti_snowplow::landmark_list foundLandmarkLocs;//vector for each found landmark location

	for(int i=0; i < landmarkLocsTXT.landmarks.size();i++)
	{
		double smallestSeparation = .75;  ////landmark_tolerance;
		//double smallestSeparation = 1.5;  ////landmark_tolerance;
		
		//Variables to store when looping through scan point cloud
		double Xsum = 0;//Summation to take average of all points from scan correlate to expected landmark position
		double ysum = 0;//Summation to take average of all points from scan correlate to expected landmark position
		double headingsum = 0;
		int found_points = 0;//used for diving the above summation to find average X and Y points of a landmark
		int k = 0;
		
		//variables for determining and saving landmark
		int LM_POINTS_THRESH = 2;//number of points a landmark must be in a scan to be considered a landmarkLocationsTxt
		//ROS_INFO_STREAM("Scanning for Landmark " << i );

		for( int j=0; j < scan_pt_cloud->points.size(); j++)
		{
			/*Check if point from Scan is close to landmark*/
			
			//ROS_INFO_STREAM("Looking at point " << i << ": (" 
			//<< scan_pt_cloud->points[i].x << ", "<< scan_pt_cloud->points[i].y) ;
			
			
			//print where point in Lidar point of view
			//ROS_INFO_STREAM("Lidar Point - X:" << scan_pt_cloud->points[j].x << "\t Y: "<<scan_pt_cloud->points[j].y<< "\tAngle: " << atan2 (scan_pt_cloud->points[j].y,scan_pt_cloud->points[j].x));
			
			//find where lidar point is in the field
			double point_magnitude= sqrt(pow(scan_pt_cloud->points[j].x,2) + pow(scan_pt_cloud->points[j].y,2));//need to find distance of point from LiDAR
			
			double point_angle_field =  prev_robot_location.heading + atan2 (scan_pt_cloud->points[j].y, scan_pt_cloud->points[j].x) - PI/2; //find the angle of the point from the X plane in the field, rather than the LiDAR	
			double point_x_field = prev_robot_location.x + cos(point_angle_field)*point_magnitude;//find the points X value in the field, not with respect to the LiDAR.
			double point_y_field = prev_robot_location.y + sin(point_angle_field)*point_magnitude;//find the points Y value in the field, not with respect to the LiDAR.
			//ROS_INFO_STREAM("Point in field - X:" << point_x_field << "\t Y: "<< point_y_field<< "\tAngle: " << point_angle_field);


			//find difference from where point is, to where landmark is expected to be
			double xSeparation = landmarkLocsTXT.landmarks[i].x - point_x_field;	
			double ySeparation = landmarkLocsTXT.landmarks[i].y - point_y_field;
			double currentSeparation = sqrt(pow(xSeparation, 2) + pow(ySeparation, 2));// find distance the current point is from the current landmark
			//ROS_INFO_STREAM("Current Seperation: "<< currentSeparation);

			if (currentSeparation < smallestSeparation)//If the point is close to the landmarks position
			{
				Xsum +=point_x_field;// add each X point to find average of X points
				ysum +=point_y_field;//add each y point to find average of y points
				headingsum += point_angle_field;//add each heading to find average heading
				found_points++;
			}
		}//end scan pointcloud loop
		//ROS_INFO_STREAM("Found " << found_points << " for landmark " <<i);

		//If scan data which we think represnts a landmark is made up of more than LM_POINT_THRESH points.
		if (found_points > LM_POINTS_THRESH) 
		{
			//Find average X, Y locaiton and average distance from expected Landmark Location
			double XAverage = Xsum/ (double)found_points;//taking average X value of found points
			double YAverage = ysum/ (double)found_points;//taking average y value of found points
			double headingAverage=headingsum/found_points;
			double distanceAverage = sqrt(XAverage*XAverage + YAverage*YAverage);
			
			//make sure the average distance from the object is not to big. IE if there is a big thing where a landmark is supposed to be
			if (distanceAverage < 15)  ////MAX_LANDMARKS_DISTANCE)
			{
				yeti_snowplow::landmark found_Landmark;
				found_Landmark.x = landmarkLocsTXT.landmarks[i].x;
				found_Landmark.y = landmarkLocsTXT.landmarks[i].y;
				found_Landmark.distance = 0.4 * distanceAverage;
                found_Landmark.distance += 0.6 * sqrt(pow(prev_robot_location.x - found_Landmark.x,2) + pow(prev_robot_location.y - found_Landmark.y,2));
				found_Landmark.heading = headingAverage;
				foundLandmarkLocs.landmarks.push_back(found_Landmark);
				//ROS_INFO_STREAM("Found landmark at " << found_Landmark.x << "\tY: " << found_Landmark.y 
				//				<< "\nAngle from 0rads: "<< found_Landmark.heading << "\tDist to landmark: " 
				//				<<found_Landmark.distance);
				continue;
			}
		}
	}//end landmark loop
	ROS_INFO_STREAM("Number of Found Landmarks: " << foundLandmarkLocs.landmarks.size());
	return foundLandmarkLocs;
}

sensor_msgs::PointCloud scanObjects(const sensor_msgs::PointCloud::ConstPtr& scan_pt_cloud, yeti_snowplow::robot_position prev_robot_location)
{
	double seperation_thresh=0.1;//max distance between points for them to be considered the same object
	double nonseperation_thresh=0.4572;//maximum distance 
	int objPointThresh = 3; // minimum number of points needed to be considered an object
	int objStartIndex=0;
	int objEndIndex;
	int linked=0;
	int object_points=0;
	int object_counter=0;
	sensor_msgs::PointCloud object_locations;

	geometry_msgs::Point32 thisPoint;
	geometry_msgs::Point32 nextPoint;

	for( int j=0; j < scan_pt_cloud->points.size()-2; j++)
	{
		thisPoint = scan_pt_cloud->points[j];
		nextPoint = scan_pt_cloud->points[j+1];

		double point_dist = sqrt( pow(thisPoint.x-nextPoint.x, 2) + pow(thisPoint.y-nextPoint.y, 2));
		if(point_dist < seperation_thresh)//points are close
		{
			ROS_INFO_STREAM("Objects found: " << object_counter);
			if(linked ==1)// already linked the object
			{
				double objSize=point_dist = sqrt( pow(thisPoint.x-scan_pt_cloud->points[objStartIndex].x, 2) + pow(thisPoint.y-scan_pt_cloud->points[objStartIndex].y, 2));
				if ( objSize > nonseperation_thresh)//object is too big to be a landmark
				{
					objEndIndex=j;
					if(objStartIndex-objEndIndex > objPointThresh)
					{
						object_counter++;
					}
				}
				object_points++;
				continue;
			}
			else
			{
				objStartIndex=j;
				int linked=1;
			}
		}
		else//points are not close
		{
			if(linked ==1)// already linked the object
			{
				objEndIndex=j;
				if(objStartIndex-objEndIndex > objPointThresh)
				{
					object_counter++;
				}
			}
			else
			{
				objStartIndex=j+1;
				continue;
			}
			linked=0;
		}
	}
	ROS_INFO_STREAM("Objects found: " << object_counter);
	return object_locations;
}

//Used to parse strings. because C++ doesn't have built in string splitting http://stackoverflow.com/a/236803
void split(const std::string &s, char delim, std::vector<std::string> &elems) 
{
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

std::vector<std::string> split(const std::string &s, char delim) 
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

//gater landmarks from text file. Text file is in ROS PARAM.
yeti_snowplow::landmark_list importLandMarks(string filename)
{
	//setup necassry string parsing and file parameters
	string str;
	ifstream file;
	file.open(filename.c_str());
	ROS_INFO("Read file line: %s", str.c_str());

	int numLandmarks = 0;
	while(getline(file, str))//check to see how many landmarks there are
	{
		numLandmarks++;
	}
	ROS_INFO("There are %d landmarks. They are: ", numLandmarks);
	file.close();

	sensor_msgs::PointCloud importedLandmarkLocations;//allocate space for landmark points
	yeti_snowplow::landmark_list importedLandmarkLocs;

	
	int landmarkNum=0;// initialize iterator
	file.open(filename.c_str());//reopen file

	while(getline(file, str))//loop through file and save landmark locations. 
	{
		//ROS_INFO("Read file line: %s", str.c_str());
		vector<string> lineFields = split(str, ' '); //x y direction PID speed
		//ROS_INFO("Line %d has %ld many fields.", landmarkNum, lineFields.size());
		
		if(lineFields.size() == 2) //ignore if too short or starts with "// "
		{ 
			yeti_snowplow::landmark found_landmarkPoint;
			found_landmarkPoint.x=atof(lineFields[0].c_str());
			found_landmarkPoint.y=atof(lineFields[1].c_str());
			ROS_INFO("Landmark %d: \tX: %f\tY:%f",landmarkNum, found_landmarkPoint.x, found_landmarkPoint.y);
			landmarkNum++;

			importedLandmarkLocs.landmarks.push_back(found_landmarkPoint);
		}
	}
	file.close();

	return importedLandmarkLocs;
}

//find where robot is in field with newly found landmark locaitons
yeti_snowplow::robot_position determineRobotLocaiton(yeti_snowplow::landmark_list FLML/*Found landmark locatinos*/, yeti_snowplow::robot_position prev_robot_location)
{
	int JMAX = 15;//maximum amount of attempts to try to find minimum error in robot location
	double mu = 0.1; // (Only read) base learning rate of least error square learning algorithm
	double maxx = 0.25;
	double minx = 0.0001;
	double maxy = 0.25;
	double miny = 0.0001;
	double maxt = 5.0 * (PI / 180.0);
	double mint = 0.1 * (PI / 180.0);

	bool updateh = false;//flag to update the hessain matrix or not
	double H[3][3]; // Hessian of 2nd Derivatives
	double H_lm[3][3];// Hessian of LM step
	double H_inv[3][3];  // Matrix Inverse of H_lm
	double x_err = 0;  // Gradient E [0]
	double y_err = 0;  // Gradient E [1] 
	double t_err = 0;  // Gradient E [2]
	double ex;  // Landmark error in X
	double ey;  // Landmark error in Y
	double dx;  // Robot Delta X
	double dy;  // Robot Delta Y 
	double dt;  // Robot Delta Theta
	double det;  // Determinant of H_lm

	double errsqrd = 0.0;  // Error of cost function squared
    double derr = 0.0; // derivative of cost function. Helps determine local minimum
    double lasterrsqrd = 99999.0;//variable to store previous error squared to determine if new error squared is better than last
    double lambda = 10.0; // Levenburg markequardt variable which changes learning rate of learning algorithm. Increases when error increases, and decreases as error decreases.
	double min_der_tolerance = .001;//tolerance of dereivative of error to determine if local minima is reached.
	yeti_snowplow::robot_position thisRobotPos = prev_robot_location;//creating variable for robot position, initializing to the previous locaiton
	
	ROS_INFO_STREAM("starting theta in localization: " << thisRobotPos.heading);
	//Initialize Hessian Matrix To Zeroes//
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			H[i][j] = 0;
			H_lm[i][j] = 0;
			H_inv[i][j] =0;
		}
	}
	
	//Attempt to minimize!
	for (int j = 0; j < JMAX; j++)
    {
		errsqrd = 0;//reset error of cost function
		derr = 0;//reset error of cost function

		if (j == 0)
		{
			updateh = true;//trigger flag to update hessian matrix.
			lasterrsqrd = 99999.0;// make last error square equal to infinity so that the first iteration the error is not thrown out.
		}
		if (updateh)
        {
			//reset hessian matrix to zeros
			for (int i = 0; i < 3; i++)
			{
				for (int k = 0; k < 3; k++)
				{
					H[i][k] = 0.0;
				}
			}

			//calculate the error variables
			H[0][0] = H[1][1] = H[2][2] = mu * FLML.landmarks.size();  // Formula 21
			x_err = mu * FLML.landmarks.size() * (prev_robot_location.x - thisRobotPos.x);
			y_err = mu * FLML.landmarks.size() * (prev_robot_location.y - thisRobotPos.y);
			t_err = mu * FLML.landmarks.size() * (prev_robot_location.heading - thisRobotPos.heading);


			for (int i = 0; i < FLML.landmarks.size();i++)
			{
				//finding error(difference) between where the landmark is, and where we expect it to be
				ex = FLML.landmarks[i].x - thisRobotPos.x - FLML.landmarks[i].distance * sin(thisRobotPos.heading + FLML.landmarks[i].heading); 
				ey = FLML.landmarks[i].y - thisRobotPos.y - FLML.landmarks[i].distance * cos(thisRobotPos.heading + FLML.landmarks[i].heading);


				//updating all of the error variables
				derr += abs(ex) + abs( ey);
				errsqrd += pow(ex, 2) + pow(ey, 2);
				x_err += ex;
                y_err += ey;
				t_err += ex * (FLML.landmarks[i].distance * cos(thisRobotPos.heading + FLML.landmarks[i].heading)) - ey * (FLML.landmarks[i].distance * sin(thisRobotPos.heading + FLML.landmarks[i].heading));
				//ROS_INFO_STREAM("t_err = " <<t_err);


				//updating hessian matrix 
				H[0][0] += 1;  // Formula 9.2, Formula 21
				H[0][1] += 0;  // Formula 10.2, Formula 21
				H[0][2] += (FLML.landmarks[i].distance * cos(thisRobotPos.heading + FLML.landmarks[i].heading));  // Formula 11.2, Formula 22
				H[1][0] += 0;  // Formula 12.2, Formula 21
				H[1][1] += 1;  // Formula 13.2,, Formula 21
				H[1][2] += -(FLML.landmarks[i].distance * sin(thisRobotPos.heading + FLML.landmarks[i].heading));  // Formula 14.2, Formula 23
				H[2][0] += (FLML.landmarks[i].distance * cos(thisRobotPos.heading + FLML.landmarks[i].heading));  // Formula 15.2, Formula 22
				H[2][1] += -(FLML.landmarks[i].distance * sin(thisRobotPos.heading + FLML.landmarks[i].heading));  // Formula 16.2, Formula 23
				H[2][2] += pow(FLML.landmarks[i].distance, 2);  // Formula 17.2, Formula 24

			}


			//create H for Levenburg marquadt method
			for (int i = 0; i < 3; i++)
			{
				for (int k = 0; k < 3; k++)
				{
					H_lm[i][k] = H[i][k];

					if (i == k)
					{
						H_lm[i][k] += lambda * 1.0;
					}
				}
			}

			//calculating determinant from LM hessian matrix
			det = H_lm[0][0] * (H_lm[2][2] * H_lm[1][1] - H_lm[2][1] * H_lm[1][2]) -
                    H_lm[1][0] * (H_lm[2][2] * H_lm[0][1] - H_lm[2][1] * H_lm[0][2]) +
                    H_lm[2][0] * (H_lm[1][2] * H_lm[0][1] - H_lm[1][1] * H_lm[0][2]);

			// Find inverse of H_lm
			H_inv[0][0] = (H_lm[2][2] * H_lm[1][1] - H_lm[2][1] * H_lm[1][2]) / det;
			H_inv[0][1] = -(H_lm[2][2] * H_lm[0][1] - H_lm[2][1] * H_lm[0][2]) / det;
			H_inv[0][2] = (H_lm[1][2] * H_lm[0][1] - H_lm[1][1] * H_lm[0][2]) / det;
			H_inv[1][0] = -(H_lm[2][2] * H_lm[1][0] - H_lm[2][0] * H_lm[1][2]) / det;
			H_inv[1][1] = (H_lm[2][2] * H_lm[0][0] - H_lm[2][0] * H_lm[0][2]) / det;
			H_inv[1][2] = -(H_lm[1][2] * H_lm[0][0] - H_lm[1][0] * H_lm[0][2]) / det;
			H_inv[2][0] = (H_lm[2][1] * H_lm[1][0] - H_lm[2][0] * H_lm[1][1]) / det;
			H_inv[2][1] = -(H_lm[2][1] * H_lm[0][0] - H_lm[2][0] * H_lm[0][1]) / det;
			H_inv[2][2] = (H_lm[1][1] * H_lm[0][0] - H_lm[1][0] * H_lm[0][1]) / det;

			//updating deltas by using Hessian Inverse times Gradiant*/
			dx = (H_inv[0][0] * x_err) + (H_inv[0][1] * y_err) + (H_inv[0][2] * t_err);  // Formula 25 (After plus)
			dy = (H_inv[1][0] * x_err) + (H_inv[1][1] * y_err) + (H_inv[1][2] * t_err);  // Formula 25 (After plus)
			dt = (H_inv[2][0] * x_err) + (H_inv[2][1] * y_err) + (H_inv[2][2] * t_err);  // Formula 25 (After plus)
			
			//check if the error is not getting sginificantly smaller
			if (abs(derr) < min_der_tolerance)
			{
				j = JMAX;  // if the error is at a minimum them use this value instead of finding a smaller one
			}
			ROS_INFO_STREAM("errorsqrd = " <<errsqrd );
			if (errsqrd < lasterrsqrd)//if error squared is better than last iteration
			{
				//ROS_INFO_STREAM("dt = " <<dt );
				updateh = true;// calculate a new position
				lambda /= 10;//make learnirng rate smalles
				lasterrsqrd = errsqrd;
				thisRobotPos.x += dx;  // Formula 25
				thisRobotPos.y += dy;  // Formula 25
				thisRobotPos.heading += dt;  // Formula 25
				ROS_INFO_STREAM("updated X is = " << thisRobotPos.x );
				ROS_INFO_STREAM("updated y is = " << thisRobotPos.y );
				ROS_INFO_STREAM("updated heading is = " << thisRobotPos.heading );
			}
			else
			{
				updateh = false;
				lambda *= 10;
			}
		}// update h
	}//end minimization attempts

	return thisRobotPos;
}


//finds landmarks, finds robot locations from landmark location, then publishes robot location
void localizeCallBack (const sensor_msgs::PointCloud::ConstPtr& laser_scan_cloud_in)
{
		
		int i = 0;
		ROS_INFO_STREAM("Previous Robot Location:\nX: "<<yeti_location.x 
						<<"\tY: "<< yeti_location.y<< "\tHeading: "<<yeti_location.heading);
		//find_where landmarks are
		yeti_snowplow::landmark_list foundLandmarkLocations;
		foundLandmarkLocations = scanLandmarks(landmarkLocationsTxt, laser_scan_cloud_in, yeti_location);

		
		//find where robot is
		yeti_location = determineRobotLocaiton(foundLandmarkLocations, yeti_location);
		ROS_INFO_STREAM("Current Robot Location:\nX: "<<yeti_location.x 
						<<"\tY: "<< yeti_location.y<< "\tHeading: "<<yeti_location.heading);
}


main (int argc, char** argv)
{
	//creating starting location for Yeti
	yeti_location.x = 0;
	yeti_location.y = 0;
	yeti_location.distance = 0;
	yeti_location.heading = PI/2;

	ros::init (argc, argv, "localization");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Finding Landmarks, and Robot Position");

	//import landmark locations from text file
	std::string landmarkLocationFile;
	if (ros::param::get("landmarkLocationFile", landmarkLocationFile))
	{
		ROS_INFO("Using landmarkLocationFile %s", landmarkLocationFile.c_str());
	}
	landmarkLocationsTxt = importLandMarks(landmarkLocationFile);
	
	
	// Create a ROS subscriber for the pointcloud published by laser geometry
 	ros::Subscriber scanSub;
 	scanSub = nh.subscribe("laser_scan_point_cloud", 1, localizeCallBack);

 	 // Create a ROS publisher for the output point cloud
 	 pub = nh.advertise<sensor_msgs::PointCloud>("robot_location", 100);

	ros::spin();
return 0;
}






