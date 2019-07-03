// myekf1.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"
#include <vector>
#include <fstream>

int main()
{
    std::cout << "Hello World!\n"; 

	// Create a Kalman Filter instance
	FusionEKF fusionEKF;

	// used to compute the RMSE later
	Tools tools;
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	MeasurementPackage meas_package;

	vector<MeasurementPackage> meas_pkg_list;
	

	string in_file_name_ = "L-R-stram.txt";
	ifstream in_file(in_file_name_.c_str(), std::ifstream::in);

	if (!in_file.is_open()) {
		cout << "Cannot open input file: " << in_file_name_ << endl;
	}

	string line;
	int i = 0;

	
    while (getline(in_file, line)&&i<100)
	{
		istringstream iss(line);
		string sensor_type;
		long long timestamp; // 是怎样的形式？  8个字节，int 是4个 ，long 是4 个
		// telemetry 事件驱动的
	    
		iss >> sensor_type;
		// for LiDAR
		if (sensor_type.compare("L") == 0)
		{
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2); // 就是一个二维的向量，记录 px 和 py
			float px;
			float py;
			iss >> px;
			iss >> py;
			meas_package.raw_measurements_ << px, py;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			meas_pkg_list.push_back(meas_package);
		}
		//  for RaDAR
		else if (sensor_type.compare("R") == 0) {

			meas_package.sensor_type_ = MeasurementPackage::RADAR;
			meas_package.raw_measurements_ = VectorXd(3);
			float ro;
			float theta;
			float ro_dot;
			iss >> ro;
			iss >> theta;
			iss >> ro_dot;
			meas_package.raw_measurements_ << ro, theta, ro_dot;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			meas_pkg_list.push_back(meas_package);
		}
		
		// ground_truth

		float x_gt;
		float y_gt;
		float vx_gt;
		float vy_gt;
		VectorXd gt_values(4);
		iss >> x_gt;
		iss >> y_gt;
		iss >> vx_gt;
		iss >> vy_gt;
		gt_values(0) = x_gt;
		gt_values(1) = y_gt;
		gt_values(2) = vx_gt;
		gt_values(3) = vy_gt;
		ground_truth.push_back(gt_values); 
		
		i++;	

	}

	size_t N = meas_pkg_list.size();
	for (size_t k = 0; k < N; ++k)
	{
		//Call ProcessMeasurment(meas_package) for Kalman filter
		fusionEKF.ProcessMeasurement(meas_pkg_list[k]); // 为何传入的是常引用？ 1 直接传递对象需要调用复制构造函数开销太大。常引用可以避免被修改

		// 后面主要是做RMRS计算评价了
	//Push the current estimated x,y positon from the Kalman filter's state vector
		VectorXd estimate(4);

		double p_x = fusionEKF.ekf_.x_(0);
		double p_y = fusionEKF.ekf_.x_(1);
		double v1 = fusionEKF.ekf_.x_(2);
		double v2 = fusionEKF.ekf_.x_(3);

		estimate(0) = p_x;
		estimate(1) = p_y;
		estimate(2) = v1;
		estimate(3) = v2;

		estimations.push_back(estimate);
	}
	
	// RMSE 估计
	VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth); 
	cout<<"RMSE: "<< RMSE <<endl;
	size_t Ne = estimations.size();
	// 输出到文件
	ofstream output_file("output_ekf.csv", ios::out | ios::app | ios::ate);
	if (output_file.is_open()) {
		cout << "Open file successfully." << endl;
		output_file << "Px" <<","<< "Py" <<","<< "Gx" <<","<< "Gy" << endl;
		for (size_t k = 0; k < Ne; ++k)
		{
			cout << "estimat_x_ = " << endl;
			cout << estimations[k] << endl;

			cout << "ground_x_ = " << endl;
			cout << ground_truth[k]<<endl;
			cout << endl;

			output_file << estimations[k][0] <<"," << estimations[k][1] << "," << ground_truth[k][0] << "," << ground_truth[k][1]<<endl;

			

		}
	
	}
	
	
}


