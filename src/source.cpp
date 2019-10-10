#include "ros/ros.h"
#include "std_msgs/String.h"

#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/JointState.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"


#include <sstream>
#include <fstream>

// #include "LPV.h"
#include "Utils.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>


bool _firstRealPoseReceived=false;

Eigen::VectorXf _eePosition(3);
//Eigen::VectorXd _eeOrientation(4);
Eigen::Matrix3f _eeRotMat;
Eigen::Vector4f _eeOrientation;

bool _firstTargetPoseReceived=false;

// Eigen::VectorXf _targetPosition(3);
Eigen::VectorXf _targetOrientation(4);
Eigen::MatrixXf _targetRotMatrix;

void robotListener(const geometry_msgs::Pose::ConstPtr& msg){

	//_msgRealPose = *msg;

	_eePosition << msg->position.x, msg->position.y, msg->position.z;
	


	if(!_firstRealPoseReceived)
	{
		_eeOrientation << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
		_eeRotMat = Utils<float>::quaternionToRotationMatrix(_eeOrientation);
		_firstRealPoseReceived = true;
		ROS_INFO("Robot Pose received\n");
  	
		
	}
}


int main(int argc, char **argv)
{

	// txt file identifier

	Eigen::VectorXf desiredNextPosition(3);
	Eigen::Vector3f _omegad;    // Desired angular velocity [rad/s] (3x1)
	Eigen::Vector4f _qd;        // Desired end effector quaternion (4x1)

	Eigen::VectorXf xD(3);		// Desired velocity

	int linesToscip=0;

	std::ifstream myfile("src/text_to_traject/txtFiles/trial_all.txt");
	if(!myfile.is_open()){
		std::cerr<<"Unable to open file\n";
		return 0;
	}

	char cNum[100];


	ros::init(argc, argv, "txtToTraj");

	ros::NodeHandle n;


	ros::Subscriber robotSub=n.subscribe("/lwr/ee_pose", 10, robotListener);

	ros::Publisher _pubDesiredTwist=n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
	ros::Publisher _pubDesiredOrientation=n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);

	// geometry_msgs::Pose _msgDesiredPose;

	geometry_msgs::Pose _msgRealPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
    geometry_msgs::Twist _msgDesiredTwist;

    _omegad.setConstant(0.0f);
    _qd.setConstant(0.0f);

	

	ros::Rate loop_rate(500);

	Eigen::MatrixXf W_M(3,3);

	W_M<< -5.0,0.0,0.0,
		  0.0,-5.0,0.0,
		  0.0,0.0,-5.0;

	int count = 0;

	while (ros::ok()){

		if(_firstRealPoseReceived){

			// update next target position

			myfile.getline(cNum,256,',');
			desiredNextPosition[0]=std::atof(cNum);
			myfile.getline(cNum,256,',');
			desiredNextPosition[1]=std::atof(cNum);
			myfile.getline(cNum,256,'\n');
			desiredNextPosition[2]=std::atof(cNum);

			// std::cerr << "A" << std::endl;
			for(int j=0;j<linesToscip;j++){
				// std::cerr << myfile.peek() << std::endl;
				if(myfile.peek() != EOF){
					// std::cerr << "j: " << j << std::endl;
					myfile.getline(cNum,256,'\n');
					// std::cerr << "j: " << j << std::endl;

				}
			}

			// std::cerr << "B" << std::endl;

			if(myfile.peek() == EOF){
							// Publish desired twist (passive ds controller)
				_msgDesiredTwist.linear.x  = 0.0f;
				_msgDesiredTwist.linear.y  = 0.0f;
				_msgDesiredTwist.linear.z  = 0.0f;
				_msgDesiredTwist.angular.x = 0.0f;
				_msgDesiredTwist.angular.y = 0.0f;
				_msgDesiredTwist.angular.z = 0.0f;

				_pubDesiredTwist.publish(_msgDesiredTwist);

				// Desired quaternion to have the end effector looking down
				_qd << 0.0f, 0.0f, 1.0f, 0.0f;

				// Publish desired orientation
				_msgDesiredOrientation.w = _qd(0);
				_msgDesiredOrientation.x = _qd(1);
				_msgDesiredOrientation.y = _qd(2);
				_msgDesiredOrientation.z = _qd(3);
				_pubDesiredOrientation.publish(_msgDesiredOrientation);
				break;
			}



			std::cout << "next target position " << desiredNextPosition[0] << " " << desiredNextPosition[1] << " " << desiredNextPosition[2] <<"\n";

			
			xD=W_M*(_eePosition-desiredNextPosition);
			if(xD.norm()>0.3f)
			{
				xD *= 0.3/xD.norm();
			}
			
			

			// xD=W_M*(_eePosition-_targetPosition);
			std::cout << "xD=" << xD[0] << " " << xD[1] << " " << xD[2] <<"\n";
			std::cout << "speed: " << xD.norm() << "\n";
			std::cout << "distance: " << (_eePosition-desiredNextPosition).norm() << "\n";
			std::cout << "target: " << desiredNextPosition[0] << " " << desiredNextPosition[1] << " " << desiredNextPosition[2] <<"\n";
			std::cout << "robot: " << _eePosition.transpose() << std::endl;

			
			// Publish desired twist (passive ds controller)
			_msgDesiredTwist.linear.x  = xD[0];
			_msgDesiredTwist.linear.y  = xD[1];
			_msgDesiredTwist.linear.z  = xD[2];
			_msgDesiredTwist.angular.x = _omegad(0);
			_msgDesiredTwist.angular.y = _omegad(1);
			_msgDesiredTwist.angular.z = _omegad(2);

			_pubDesiredTwist.publish(_msgDesiredTwist);

			// Desired quaternion to have the end effector looking down
			_qd << 0.0f, 0.0f, 1.0f, 0.0f;

			// Publish desired orientation
			_msgDesiredOrientation.w = _qd(0);
			_msgDesiredOrientation.x = _qd(1);
			_msgDesiredOrientation.y = _qd(2);
			_msgDesiredOrientation.z = _qd(3);
			_pubDesiredOrientation.publish(_msgDesiredOrientation);

	
		}
		
		
		// std_msgs::String msg;
		// std::stringstream ss;
		// ss << "hello world " << count;
		// msg.data = ss.str();

		// ROS_INFO("%s", msg.data.c_str());
		//chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}
	myfile.close();

	return 0;

}
