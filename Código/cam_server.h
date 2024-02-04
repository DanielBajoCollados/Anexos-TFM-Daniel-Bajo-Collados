#ifndef SERVER_H
#define SERVER_H

#include <QObject>
#include <QDebug>
#include <QTcpServer>
#include <QTimer>
#include <qtcpsocket.h>
#include <opencv2/core.hpp>

class Server : public QObject
{
    Q_OBJECT
public:
    explicit Server(QObject *parent = nullptr);
    ~Server();
    bool saveImg(QByteArray img_array, std::string fileName); //Save an image to a file
    int detectAruco(std::string fileName, std::vector<int> &markerIds, std::vector<cv::Vec3d> &rVectors, std::vector<cv::Vec3d> &tVectors, bool savePos = true); // Detect ArUco markers from an image.
        // Returns the number of markers detected, or -1 on error
    bool sendRequest(QTcpSocket *socket); // Send a request for a picture to the client
    cv::Mat estimateModulePose(std::vector<int> markerIds, std::vector<cv::Vec3d> rVectors, std::vector<cv::Vec3d> tVectors, std::string fileName, bool saveFinds = true); //Estimates the pose of the module to attach to with the Aruco Positions set as input.
        // Returns Transformation Matrix from the origin of the coupler to the rear of the module to attach to
    std::vector<double> forwardKinematics(double* input);   //Forward kinematics of the coupling arm
    void inverseKinematics(double* input, double* output);   //Inverse kinematics of the coupling arm

signals:

public slots:
    void newConnection();//Function for when there is a new connection

private:
    QTcpServer *server;
    const size_t msg_size = 1024;
    bool loadCameraCalib(std::string name, cv::Mat& cameraMatrix, cv::Mat& distanceCoefficients);   // Load camera calibration from file
};

#endif // SERVER_H
