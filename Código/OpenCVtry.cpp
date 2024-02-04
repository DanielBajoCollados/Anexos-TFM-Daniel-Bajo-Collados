#include <Windows.h>
#include <conio.h>
#include <dos.h>


#include "opencv2/core.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/objdetect.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdint.h>

using namespace std;
using namespace cv;
const int fps = 60;

const float ArucoSqareDimension = 0.043;

//Global Functions

bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficient) {
	ifstream inStream(name);
	if (inStream) {
		uint16_t rows, columns;

		inStream >> rows >> columns;

		cameraMatrix = Mat(Size(columns, rows), CV_64F);

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				double read = 0.0f;
				inStream >> read;
				cameraMatrix.at<double>(r, c) = read;
			}
		}

		inStream >> rows >> columns;

		distanceCoefficient = Mat(Size(columns, rows), CV_64F);

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				double read = 0.0f;
				inStream >> read;
				distanceCoefficient.at<double>(r, c) = read;
			}
		}

		inStream.close();
		return true;
	}


	return false;

}

//Get a Transformation matrix from a Rotation & Traslation Vector
Mat getTMat(Vec3d rvec, Vec3d tvec) {
	Mat TMat = Mat::zeros(4, 4, CV_64F), RMat = Mat::zeros(3, 3, CV_64F);//Create the tempate for both the transformation and rotation matrices

	//cout << rvec << "   " << tvec << endl;//Print the values of poth rotation and traslation vectors

	Rodrigues(rvec, RMat);//Get the values for the rotation matrix

	//cout << Rmat << endl;//Print the rotation matrix

	//Get the values of the transformation matrix via rotation matrix and traslation vector
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			TMat.at<double>(i, j) = RMat.at<double>(i, j);
		}
	}
	for (int i = 0; i < 3; i++) {
		TMat.at<double>(i, 3) = tvec.val[i];
	}
	TMat.at<double>(3, 3) = 1;
	//cout << "T Matrix is:" << endl << TMat << endl;//Print the traslation matrix
	return TMat;
}

//Get a rotation and traslation Vector from a Transformation Matrix
void getRTVec(Mat TMat, Vec3d& RVec, Vec3d& TVec) {
	for (int i = 0; i < 3; i++)
		TVec[i] = TMat.at<double>(i, 3);
	Mat RAux(3, 3, CV_64F);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			RAux.at<double>(i, j) = TMat.at<double>(i, j);
	}
	Rodrigues(RAux, RVec);

	//cout << "TVec is:" << endl << TVec << endl << "RVec is:" << endl << RVec << endl;
}

//Load the data of the matrices needed to calculate the position of the base of the Aruco Pyramid
bool loadOriginTMatrix(string name, Mat& OriginTMatrix, VideoCapture vid, Mat CameraMatrix, Mat DistanceCoefficients) {

	//Calculate origin matrix with the camera
	if (!vid.isOpened()) {
		cout << "ERROR: Couldn't get an image from camera. Unable to get the value of the origin matrix" << endl;
		return (0);
	}

	Mat frame;
	vector<vector<Point2f>> goodCorners;
	aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
	cv::aruco::ArucoDetector detector(dictionary, detectorParams);
	vector<int> markerId;

	while (vid.read(frame)) {

		cout << "Searching for Aruco marker to set origin matrix" << endl;

		detector.detectMarkers(frame, goodCorners, markerId);

		if (markerId.size() > 0) {

			vector <Vec3d> rvector, tvector;

			aruco::estimatePoseSingleMarkers(goodCorners, ArucoSqareDimension, CameraMatrix, DistanceCoefficients, rvector, tvector);

			OriginTMatrix = getTMat(rvector[0], tvector[0]).inv();

			/*/std::cout << "Saving File..." << std::endl;
			cv::Mat markersDetected = frame.clone();
			std::cout << "Draw detected Markers..." << std::endl;
			cv::aruco::drawDetectedMarkers(markersDetected, goodCorners, markerId);
			std::cout << "Draw frame axes..." << std::endl;
			cv::drawFrameAxes(markersDetected, CameraMatrix, DistanceCoefficients, rvector, tvector, 0.01, 2);

			imwrite("../Frame.jpg", markersDetected);*/

			break;
		}
	}

	return (1);
}

//Send message via communication file
bool sendMsg(string name, char outbuff) {

	ofstream CommFile;

	CommFile.open(name, ios::out | ios::trunc);

	if (!CommFile) {

		cout << "ERROR: Could not open CommunicationFile.txt" << endl;
		return 0;
	}

	CommFile.seekp(0);

	CommFile.put(outbuff);

	CommFile.close();

	return 1;
}

//Recieve message via communication file
bool recvMsg(string name, char& inbuff) {

	fstream CommFile;

	//Read the character

	CommFile.open(name, ios::in);
	if (!CommFile) {

		printf("ERROR: Could not open CommunicationFile.txt\n");
		return 0;
	}

	CommFile.seekg(0);

	CommFile.get(inbuff);

	CommFile.close();

	Sleep(100);

	return 1;
}

int main()
{
	//File to communicate with the motors control process

	string CommFile = "../CommunicationFile.txt";

	Mat frame, outputImg;

	vector<vector<Point2f>> goodCorners;
	aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
	cv::aruco::ArucoDetector detector(dictionary, detectorParams);
	vector<int> markerId;

	Mat cameramatrix = Mat::eye(3, 3, CV_64F);
	Mat distanceCoefficients;

	vector <Mat> savedImages;

	loadCameraCalibration("CameraCalibration", cameramatrix, distanceCoefficients);

	vector<vector<Point2f>> markerCorners;

	VideoCapture vid = VideoCapture(0, CAP_DSHOW);

	if (!vid.isOpened())
		return -1;

	//Get the values of all origin Matrices
	Mat TMatOr = Mat::zeros(4, 4, CV_64F);//Origin T matrix
	if (!loadOriginTMatrix("Matrices.csv", TMatOr, vid, cameramatrix, distanceCoefficients)) {
		cout << "An error occured while setting all origin matrices for future measurements" << endl;
		return (-1);
	}

	int step = 0;

	while (1)
	{

		//Send signal to MotorsControl to start working

		if (!sendMsg(CommFile, 's')) {
			cout << "ERROR: Could not send mesage" << endl;
			return -1;
		}

		//Wait until MotorsControl sends a signal to let know the final position has been reached

		while (1) {
			char inBuff;

			recvMsg(CommFile, inBuff);

			if (inBuff == 'r')
				break;

			if (inBuff == 'e') {//If MotorsControl sends an error signal

				cout << "ERROR: MotorsControl has encountered a problem" << endl << "Close and restart" << endl;

				//clear comm file
				ofstream clearFile;
				clearFile.open(CommFile, ios::out | ios::trunc);
				clearFile.close();

				return -1;
			}
		}

		vector<Vec3d> rvector, tvector;
		Mat TMat = Mat::zeros(4, 4, CV_64F);
		bool Obtainedflag = 1; //Flag to state that the position was obtained and is not bad

		vid.read(frame);

		outputImg = frame.clone();

		detector.detectMarkers(frame, goodCorners, markerId);

		aruco::drawDetectedMarkers(outputImg, goodCorners, markerId);

		aruco::estimatePoseSingleMarkers(goodCorners, ArucoSqareDimension, cameramatrix, distanceCoefficients, rvector, tvector);

		Mat Identity = cv::Mat::eye(3, 3, CV_64F);
		Vec3d Identityvec;
		Rodrigues(Identity, Identityvec);

		if (markerId.size() > 0) //If the position can be obtained
			TMat = TMatOr * getTMat(rvector[0], tvector[0]);

		else
			Obtainedflag = 0;


		//Insert the values on the Data File
		if (Obtainedflag) {//If the position was obtained

			Obtainedflag = 0;
			cout << "T Matrix obtained is: \n" << TMat << endl;

			ofstream DataFile;//file to store all obtained data for the neural network

			//If the Z value is negative, an error has occurred, sice the disc cannot be underneath the robot 
			//Therefore the mesauremente is incorrect
			//This also happens when the X or Y values are beyond +-120 mm, which is further that what the arm can reach
			if ((TMat.at<double>(2, 3) < 0) || (abs(TMat.at<double>(1, 3)) > 0.120) || (abs(TMat.at<double>(0, 3)) > 0.120)) {
				//Send mesage to set new position
				cout << "Measurement failed. The position seems outside boundaries. \n\t\tSet another position" << endl;
				if (!sendMsg(CommFile, 's')) {
					cout << "ERROR: Could not send mesage" << endl;
					return -1;
				}
			}

			else {
				stringstream strDataBuffer;
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 4; j++) {
						strDataBuffer << TMat.at<double>(i, j) << "; ";
					}
				}

				long size = strDataBuffer.str().size() + 1;
				char* charDataBuffer = new char[size];
				strcpy_s(charDataBuffer, size, strDataBuffer.str().c_str());

				cout << "String: " << strDataBuffer.str() << endl << "Char: " << charDataBuffer << endl;

				DataFile.open("../Data File.csv", ios::app);//Opens the .csv file and sets the cursor at the end
				DataFile.write(charDataBuffer, size);
				DataFile.close();

				//Send mesage to write motors positions on DataFile
				if (!sendMsg(CommFile, 'w')) {
					cout << "ERROR: Could not send mesage" << endl;
					return -1;
				}

				step++;
				cout << "\tMeasurement number: " << step << endl;
			}
		}
		else {//If the position cannot be obtained because there are no ArUco markers available

			cout << "No valid ArUco marker was found. \n\t\tSet another position" << endl;
			//Send mesage to set new position
			if (!sendMsg(CommFile, 's')) {
				cout << "ERROR: Could not send mesage" << endl;
				return -1;
			}
		}

		//Wait until MotorsContol says it has either written all motors positions 
		//or has recieved the signal stating ArUcoRead is unable to cakculate the new position
		while (1) {
			char inBuff;

			recvMsg(CommFile, inBuff);

			if (inBuff == 'o')
				break;
		}

	}

	if (!sendMsg(CommFile, 'q')) {
		cout << "ERROR: Could not send mesage" << endl;
		return -1;
	}
	return 0;
}