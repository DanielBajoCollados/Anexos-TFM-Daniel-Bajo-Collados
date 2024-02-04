#include "cam_server.h"
#include <QSaveFile>
#include <QDir>
#include <QTimer>
#include <QtMath>

#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/core/quaternion.hpp>

//Definitions
#define TIMEOUT 10000       //Time limit for package reception in ms
#define PORT 11000          //Port of communication

double markerSize = 0.037f; //Size of the ArUco Markers in m

int image_num=1;            //Number of images recieved

Server::Server(QObject *parent)
    : QObject{parent}
{
    server = new QTcpServer(this);

    connect(server, SIGNAL(newConnection()), this, SLOT(newConnection()));

    if(!server->listen(QHostAddress::Any, PORT)){
        std::cout << "Server could not start!" << std::endl;
    }
    else{
        std::cout << "Server started" << std::endl;
    }

}

Server::~Server(){
    disconnect(server, SIGNAL(newConnection()), this, SLOT(newConnection()));
}


void Server::newConnection(){

    QTcpSocket *socket = server->nextPendingConnection();

    std::cout << "newConnection: Socket Port is: " << socket->localPort() << std::endl;

    QByteArray img_sizeQT;    //Size of the image that will be sent

    socket->waitForReadyRead();
    img_sizeQT = socket->read(4);

    //std::cout << "Img size recieved" << std::endl;

    size_t *img_size = (size_t*)img_sizeQT.data();
    //std::cout << *img_size << std::endl; //Pointer to size

    socket->write("size recieved\r\n");

    socket->flush();

    uint8_t pck_no = *img_size/msg_size;
    size_t rest = *img_size%msg_size;

    //std::cout << "Number of packages: " << pck_no << " Rest is: " << rest << std::endl;


    QByteArray img_array;//Array in which the image will be saved
    img_array.reserve(*img_size);

    //////////////////////////////////////////////////////////////////////
    for (int i=0; i<pck_no; i++){
        if (!socket->waitForReadyRead(TIMEOUT)){
            std::cout << "new Connection: Timeout error" << std::endl;
            socket->close();
            return;
        }

        img_array += socket->read(msg_size);

        //img_array.size();

        socket->write("pkg recieved\r\n");

        socket->flush();
    }

    if(rest){
        if (!socket->waitForReadyRead(TIMEOUT)){
            std::cout << "new Connection: Timeout error" << std::endl;
            socket->close();
            return;
        }

        img_array += socket->read(rest);

        socket->write("last pkg recieved\r\n");

        socket->flush();
    }

    if (!socket->waitForReadyRead(TIMEOUT)){
        std::cout << "new Connection: Timeout error" << std::endl;
        socket->close();
        return;
    }

    QByteArray Request = socket->read(1);
    if (Request.isEmpty()){
        std::cout << "New Connection: Didn't recieve proper request" << std::endl;
        double output[6];

        QByteArray motorsPosition;

        motorsPosition.append((char *)output, sizeof(double)*6);

        socket->write(motorsPosition);
        socket->flush();
        socket->close();
        return;
    }

    std::string fileName = "Frame";
    fileName.append(QString::number(image_num).toStdString());
    fileName.append(".jpg");
    if (!saveImg(img_array, fileName)){
        std::cout << "New Connection: Error when saving the file" << std::endl;
        double output[6];

        QByteArray motorsPosition;

        motorsPosition.append((char *)output, sizeof(double)*6);

        socket->write(motorsPosition);
        socket->flush();
        socket->close();
        return;
    }

    //Detect all posible ArUco Markers
    std::vector<int> markerIds;
    std::vector<cv::Vec3d> tVectors, rVectors;
    int numMarkers=detectAruco(fileName, markerIds, rVectors, tVectors);
    if (numMarkers<=0){
        std::cout << "New Connection: No Markers detected" << std::endl;
        double output[6];
        QByteArray motorsPosition;
        motorsPosition.append((char *)output, sizeof(double)*6);
        socket->write(motorsPosition);
        socket->flush();
        socket->close();
        return;
    }

    /*for (int i = 0; i < markerIds.size(); i++)
        std::cout << markerIds.at(i) << ", ";
    std::cout << std::endl << "New Connection: TVectors: ";
    for (cv::Vec3d i: tVectors)
        std::cout << i << ", ";
    std::cout << std::endl << "New Connection: RVectors: ";
    for (cv::Vec3d i: rVectors)
        std::cout << i << ", ";
    std::cout << std::endl;*/

    //Detect the position of the module
    cv::Mat TMat = estimateModulePose(markerIds, rVectors, tVectors, fileName);
    
    //Convert Matrix to Quaternion and Position
    cv::Mat RMat = cv::Mat::ones(3, 3, CV_64F);
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++)
            RMat.at<double>(i,j)=TMat.at<double>(i,j);
    }
    cv::Quat<double> Quaternion = cv::Quat<double>::createFromRotMat(RMat);
    
    double input[7] = {Quaternion[0], Quaternion[1], Quaternion[2], Quaternion[3], TMat.at<double>(0, 3), TMat.at<double>(1, 3), TMat.at<double>(2, 3)}, output[6];

    inverseKinematics(input, output);

    for (int i=0; i<7; i++)
        std::cout << input[i] << " ";
    std::cout << std::endl;

    for (int i=0; i<6; i++)
        std::cout<< output[i] << " ";
    std::cout << std::endl;

    QByteArray motorsPosition;

    motorsPosition.append((char *)output, sizeof(double)*6);

    socket->write(motorsPosition);
    socket->flush();
    
    image_num++;

    std::cout << "New Connection: Closing socket" << std::endl;
    socket->close();
    std::cout << "New Connection: Socket closed" << std::endl;
    return;
}



int Server::detectAruco(std::string fileName, std::vector<int> &markerIds, std::vector<cv::Vec3d> &rVectors, std::vector<cv::Vec3d> &tVectors, bool saveFinds){

    QDir cur_path = QDir::currentPath();    //Current Path
    QString file_name = cur_path.path();    //Name of the file, including path
    file_name.append("/../TCPTest/images/");
    file_name.append(QString::fromStdString(fileName));

    cv::Mat image = cv::imread(file_name.toStdString(), 1);
    if (image.data == NULL){
        std::cout << "Error when reading the image" << std::endl;
        return -1;
    }
    std::cout<<"Obtained image"<<std::endl;
    std::cout<<"File: " << file_name.toStdString() << std::endl;

    cv::Mat cameraMatrix, distanceCoefficients;
    loadCameraCalib("D:/ROBOMINERS/TCPTest/cameraCalibration.txt", cameraMatrix, distanceCoefficients);

    //std::cout<<"Camera Matrix:"<<std::endl<<cameraMatrix<<std::endl<<"Distance Coefficients"<<std::endl<<distanceCoefficients<<std::endl;

    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::Dictionary ArucoDic = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(ArucoDic, detectorParams);
    detector.detectMarkers(image, markerCorners, markerIds, rejectedCandidates);
    std::cout << "Markers detected: " << markerIds.size() << std::endl;
    if (markerIds.size()>0){
        cv::aruco::estimatePoseSingleMarkers(markerCorners, markerSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
        std::cout << "Pose estimated" << std::endl;

        if (saveFinds){
            std::cout<<"Saving File..." << std::endl;
            cv::Mat markersDetected = image.clone();
            std::cout << "Draw detected Markers..." << std::endl;
            cv::aruco::drawDetectedMarkers(markersDetected, markerCorners, markerIds);
            std::cout << "Draw frame axes..." << std::endl;
            for (int i=0; i<markerIds.size(); i++)
                cv::drawFrameAxes(markersDetected, cameraMatrix, distanceCoefficients, rVectors[i], tVectors[i], markerSize/2, 2);

            QString detectedFileName = file_name;
            int index=detectedFileName.lastIndexOf(".jpg");
            detectedFileName.remove(index, (detectedFileName.size()-index));
            detectedFileName.append("Detected.jpg");
            std::cout<<"Saving file: " << detectedFileName.toStdString() << "..." << std::endl;
            if (!cv::imwrite(detectedFileName.toStdString(), markersDetected)){
                std::cout<<"Error when saving image file" << std::endl;
                return -1;
            }

            std::cout<<"file closed" << std::endl;

            QString detectedPosFileName = file_name;
            detectedPosFileName.remove(index, (detectedFileName.size()-index));
            detectedPosFileName.append("markersPos.txt");


            std::cout<<"Saving file: " << detectedPosFileName.toStdString() << "..." << std::endl;
            std::ofstream outStream(detectedPosFileName.toStdString());

            if (!outStream){
                std::cout<<"Error when opening file" << std::endl;
                return markerIds.size();
            }
            for (cv::Vec3d i: tVectors)
                outStream << i << ", ";
            outStream << std::endl;
            for (cv::Vec3d i: rVectors)
                outStream << i << ", ";
            outStream.close();
        }
    }

    return markerIds.size();
}

cv::Mat Server::estimateModulePose(std::vector<int> markerIds, std::vector<cv::Vec3d> rVectors, std::vector<cv::Vec3d> tVectors, std::string fileName, bool savePos){
    if (markerIds.size()<=0){
        std::cout << "Error: Impossible to estimate Module position. There are no markers" << std::endl;
        return cv::Mat();
    }
    cv::Vec3d tVec, rVec;
    cv::Mat AuxTMat = cv::Mat::zeros(4, 4, CV_64F);
    for (int i=0; i<markerIds.size(); i++){//Run through all the markers found
        cv::Mat TMat = cv::Mat::zeros(4, 4, CV_64F), RMat = cv::Mat::zeros(3, 3, CV_64F);

        cv::Rodrigues(rVectors[i], RMat);  // Convert Rotation Vector into rotation matrix

        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                TMat.at<double>(j, k) = RMat.at<double>(j, k);
            }
        }
        for (int j = 0; j < 3; j++) {
            TMat.at<double>(j, 3) = tVectors[i][j];
        }
        TMat.at<double>(3, 3) = 1;

        //std::cout << "TMat:" << std::endl << TMat << std::endl;


        cv::Mat ArUco2Rear = cv::Mat::eye(4, 4, CV_64F);  // Transform matrix from ArUco code to rear of the module
        //Linear movement of -87.9mm on the y axis
        ArUco2Rear.at<double>(1, 3) = -0.0879f;
        //Linear movement of 5.1mm on the z axis
        ArUco2Rear.at<double>(2, 3) = 0.0051f;
        //Scale factor
        ArUco2Rear.at<double>(3, 3) = 1;

        cv::Mat AuxR1 = cv::Mat::eye(3, 3, CV_64F), AuxR2 = cv::Mat::eye(4, 4, CV_64F);//Auxiliary Rotation Matrices
        cv::Vec3d VecR2(0, 0, 0);
        //Rotation of 180º on the x axis
        VecR2[0]=3.141592f;
        cv::Rodrigues(VecR2, AuxR1);

        //Introduce the rotation matrix into the transformation matrix
        for (int j=0; j<3; j++){
            for (int k=0; k<3; k++){
                ArUco2Rear.at<double>(j, k)=AuxR1.at<double>(j, k);
            }
        }
        switch (markerIds[i]){
        case 0:
            TMat = TMat*ArUco2Rear;   //From the position of the Marker, it moves to the rear end of the module and a last rotation for cases of Id 1 and 2
            break;
        case 1:
            // Rotation of -120º on the z axis
            VecR2[0]=0;
            VecR2[1]=0;
            VecR2[2]=-2.094395f;
            cv::Rodrigues(VecR2, AuxR1);

            for (int j=0; j<3; j++){
                for (int k=0; k<3; k++){
                    AuxR2.at<double>(j, k)=AuxR1.at<double>(j, k);
                }
            }
            TMat = TMat*ArUco2Rear*AuxR2;   //From the position of the Marker, it moves to the rear end of the module and a last rotation for cases of Id 1 and 2
            break;
        case 2:
            // Rotation of 120º on the z axis
            VecR2[0]=0;
            VecR2[1]=0;
            VecR2[2]=2.094395f;
            cv::Rodrigues(VecR2, AuxR1);

            for (int j=0; j<3; j++){
                for (int k=0; k<3; k++){
                    AuxR2.at<double>(j, k)=AuxR1.at<double>(j, k);
                }
            }
            TMat = TMat*ArUco2Rear*AuxR2;   //From the position of the Marker, it moves to the rear end of the module and a last rotation for cases of Id 1 and 2
            break;
        default:
            std::cout << "Error: Marker Id doesn't correspond to the ones on the module" << std::endl;
            break;
        }


        //std::cout << "ArUco2Rear:" << std::endl << ArUco2Rear << std::endl << "TMat:" << std::endl << TMat << std::endl;

        for (int j=0; j<4; j++){
            for (int k=0; k<4; k++){
                AuxTMat.at<double>(j, k)+=TMat.at<double>(j, k)/markerIds.size();
            }
        }
    }

    cv::Mat AuxR = cv::Mat::eye(3, 3, CV_64F);
    for (int j=0; j<3; j++){
        tVec[j]=AuxTMat.at<double>(j, 3);
    }
    for (int j=0; j<3; j++){
        for (int k=0; k<3; k++){
            AuxR.at<double>(j, k)=AuxTMat.at<double>(j, k);
        }
    }
    cv::Rodrigues(AuxR, rVec);
    //std::cout << "TVec: " << tVec << std::endl << "RVec: " << rVec << std::endl;

    if(savePos){
        std::cout<<"Saving File..." << std::endl;
        cv::Mat cameraMatrix, distanceCoefficients;
        loadCameraCalib("D:/ROBOMINERS/TCPTest/cameraCalibration.txt", cameraMatrix, distanceCoefficients);
        QDir cur_path = QDir::currentPath();    //Current Path
        QString detectedFileName = cur_path.path();    //Name of the file, including path
        detectedFileName.append("/../TCPTest/images/");
        detectedFileName.append(QString::fromStdString(fileName));

        int index=detectedFileName.lastIndexOf(".jpg");
        detectedFileName.remove(index, (detectedFileName.size()-index));
        detectedFileName.append("Detected.jpg");
        cv::Mat markersDetected = cv::imread(detectedFileName.toStdString());
        cv::drawFrameAxes(markersDetected, cameraMatrix, distanceCoefficients, rVec, tVec, markerSize/2, 2);

        index=detectedFileName.lastIndexOf(".jpg");
        detectedFileName.remove(index, (detectedFileName.size()-index));
        detectedFileName.append("Module");
        detectedFileName.append(".jpg");
        std::cout<<"Saving file: " << detectedFileName.toStdString() << "..." << std::endl;
        if (!cv::imwrite(detectedFileName.toStdString(), markersDetected)){
            std::cout<<"Error when saving image file" << std::endl;
            return cv::Mat();
        }

        std::cout<<"file closed" << std::endl;
    }

    //Convert TVec and RVec to TMat
    cv::Mat TMat=cv::Mat::eye(4, 4, CV_64F), AuxR1=cv::Mat::zeros(3, 3, CV_64F);
    for (int i=0; i<3; i++){
        TMat.at<double>(i, 3)=tVec[i];
    }
    cv::Rodrigues(rVec, AuxR1);
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            TMat.at<double>(i, j)=AuxR1.at<double>(i, j);
        }
    }

    //Calculate TMat to end of leg
    double R=0.0788f;
    cv::Mat EndLegMat = cv::Mat::zeros(4, 4, CV_64F), LegMat = cv::Mat::eye(4, 4, CV_64F);
    EndLegMat.at<double>(2,0)=EndLegMat.at<double>(0,2)=EndLegMat.at<double>(3,3)=1;
    EndLegMat.at<double>(1,1)=-1;
    EndLegMat.at<double>(0,3)=0.0329f;
    EndLegMat.at<double>(1,3)=0.0121f;
    LegMat.at<double>(0,0)=-1;
    LegMat.at<double>(0,0)=-R;
    LegMat.at<double>(2,2)=-1;

    //Calculate TMat to Start of Leg
    cv::Mat OrLegMat = cv::Mat::eye(4, 4, CV_64F);
    cv::Vec3d q, d, e, f, L1, L2, x0;

    //Rotation of each degree of freedom in radians
    q[0]=3.4759;
    q[1]=-0.5009;
    q[2]=3.4809;
    //Lengths of the links
    L1[0]=0.0385f;
    L1[1]=0.055f;
    L1[2]=0.0385f;
    L2[0]=0.042f;
    L2[1]=0.06f;
    L2[2]=0.042f;
    //Positions of the axis
    x0[0]=0;
    x0[1]=0.0338f;
    x0[2]=0.0788f;

    double u1, v1, w1, u2, v2, w2, phi, Bx, By;

    for (int i=0; i<3; i++){
        d[i]=-2*(x0[i]+L1[i]*qCos(q[i]));
        e[i]=-2*L1[i]*qSin(q[i]);
        f[i]=qPow((x0[i]+L1[i]*qCos(q[i])),2)+qPow((L1[i]*qSin(q[i])),2)-qPow(L2[i],2);
    }

    u1=qPow(d[2]-d[1], 2)+qPow(e[2]-e[1], 2);
    v1=2*(f[2]-f[1])*(e[2]-e[1])-(d[1]*(d[2]-d[1])*(e[2]-e[1]))+e[1]*qPow(d[2]-d[1], 2);
    w1=qPow(f[2]-f[1], 2)-(d[1]*(f[2]-f[1])*(d[2]-d[1]))+f[1]*qPow(d[2]-d[1], 2);

    By = (-v1+qSqrt(qPow(v1, 2)-4*u1*w1))/(2*u1);
    Bx = - ((e[2]-e[1])*By + (f[2]-f[1]))/(d[2]-d[1]);

    u2 = -2*R*(Bx-L1[0]*qCos(q[0]));
    v2 = -2*R*(By-L1[0]*qSin(q[0]));
    w2 = qPow((Bx-L1[0]*qCos(q[0])), 2) + qPow((By-L1[0]*qSin(q[0])), 2) - qPow(L2[0], 2) + qPow(R, 2);

    phi=2/tan((-v2-qSqrt(qPow(v2,2)-qPow(w2,2)+qPow(u2,2)))/(w2-u2));

    OrLegMat.at<double>(0,0)=qCos(phi);
    OrLegMat.at<double>(0,1)=-qSin(phi);
    OrLegMat.at<double>(0,3)=Bx;
    OrLegMat.at<double>(1,0)=qSin(phi);
    OrLegMat.at<double>(1,1)=qCos(phi);
    OrLegMat.at<double>(1,3)=By;

    //Calculate TMat to Start of coupler
    cv::Mat OrCoupMat = cv::Mat::eye(4, 4, CV_64F);
    OrCoupMat.at<double>(0,0)=0;
    OrCoupMat.at<double>(0,2)=-1;
    OrCoupMat.at<double>(0,3)=0.091225f;
    OrCoupMat.at<double>(1,3)=0.099207f;
    OrCoupMat.at<double>(2,0)=1;
    OrCoupMat.at<double>(2,2)=0;

    std::cout << TMat << std::endl << OrCoupMat << std::endl << OrLegMat << std::endl << LegMat << std::endl << EndLegMat << std::endl;
    //std::cout << "Unite matrices" << std::endl;
    TMat=OrCoupMat*OrLegMat.inv()*LegMat*EndLegMat*TMat;

    return cv::Mat(TMat);
}

bool Server::loadCameraCalib(std::string name, cv::Mat& cameraMatrix, cv::Mat& distanceCoefficients){
    std::ifstream inStream(name);
    if (inStream){
        uint16_t rows;
        uint16_t cols;

        inStream >> rows;
        inStream >> cols;

        cameraMatrix = cv::Mat::zeros(cv::Size(cols, rows), CV_64F);

        for (int i = 0; i<rows; i++){
            for (int j=0; j<cols; j++){
                double read = 0.0f;
                inStream >> read;
                cameraMatrix.at<double>(i,j) = read;
                //std::cout << cameraMatrix.at<double>(i,j) << std::endl;
            }
        }



        inStream >> rows;
        inStream >> cols;

        distanceCoefficients = cv::Mat::zeros(cv::Size(cols, rows), CV_64F);

        for (int i = 0; i<rows; i++){
            for (int j=0; j<cols; j++){
                double read = 0.0f;
                inStream >> read;
                distanceCoefficients.at<double>(i,j) = read;
                //std::cout << distanceCoefficients.at<double>(i,j) << std::endl;
            }
        }

        std::cout << "Calibration Matrix obtained" << std::endl;
        return true;
    }
    std::cout << "Error when accessing the calibration matrix" << std::endl;
    return false;
}

bool Server::saveImg(QByteArray img_array, std::string fileName){

    QDir cur_path = QDir::currentPath();    //Current Path
    QString file_name = cur_path.path();    //Name of the file, including path
    file_name.append("/../TCPTest/images/");
    file_name.append(QString::fromStdString(fileName));

    QSaveFile img_file(file_name);
    if (!img_file.open(QIODevice::WriteOnly)){
        std::cout<<"Error when opening file" << std::endl;
        return false;
    }

    if(!img_file.isOpen()){
        std::cout<<"Error: File is not open" << std::endl;
        return false;
    }

    std::cout<<"File "<<img_file.fileName().toStdString()<<" opened. size is: "<<img_array.size() << std::endl;

    img_file.setDirectWriteFallback(true);

    //QDataStream img_stream(&img_array, QIODevice::WriteOnly);

    qint64 bytesWritten = img_file.write(img_array);
    if (-1 == bytesWritten){
        std::cout<<"Error when writing file" << std::endl;
        return false;
    }

    std::cout <<"Wrote "<<bytesWritten<<" bytes" << std::endl;

    if (!img_file.commit()){
        std::cout<<"Error when saving file" << std::endl;
        return false;
    }

    std::cout<<"file closed" << std::endl;
    return true;
}


std::vector<double> Server::forwardKinematics(double* input){
    std::vector<double> output;

    double mean_in[6] = {}, std_in[6] = {}, mean_out[7] = {}, std_out[7] = {};
    double datab[7*150]={0.991884884192775, -0.890577553398753, 0.303151920731279, -0.688383314405608, -0.482099880482161, 0.855636536433747, -0.96286181964556, -0.42312725171697, -0.30088511904896, -0.118661507880907, 0.348852629048947, -0.976607040066352, -0.927259409003027, -0.0577880057122744, -0.958519060550673, 0.758177935742041, 0.170345028803997, 0.446039987951073, -0.882107667631975, 0.598211204036863, 0.672936600466981, -0.214694036024758, 0.326000651745743, 0.0603262987888284, 0.0989790280456639, -0.138955815074805, -0.428148534871135, 0.147965809022332, 0.701179569607957, -0.285446024320511, -0.824435303194743, -0.873174548367471, 0.328751297720717, 0.797742645929019, -0.452570348408358, -0.463936138696144, -0.667712350644414, 0.40249504827244, -0.793806330071855, 0.716763083229425, 0.0344241954214639, -0.409135732884365, -0.360523545463033, -0.989763233534631, -0.123452760281895, -0.616391502896389, -0.948918117182882, 0.505525209583435, -0.893619002457522, -0.990716816686224, -0.365587079473211, 0.670017935682863, -0.823937009255188, -0.146439693243625, -0.856100127908608, 0.192937024670257, -0.460701730605614, 0.494431638805533, -0.24985797876472, 0.528093226478443, -0.281170264202655, 0.0844846464786781, 0.000385503535995202, 0.102469815625532, 0.254497445355806, -0.97775427998338, 0.883190612401151, -0.0959748238724925, -0.867729329531647, -0.948574741553746, 0.379787988072555, -0.739121895511286, 0.0525064417090559, -0.785924944569512, 0.137827864049622, -0.187923196330053, 0.627928819132752, 0.94824944066475, 0.806394662710225, 0.156912063939089, -0.429898837648613, 0.308128666043254, 0.490699445331902, 0.534215441733418, 0.640192674952186, -0.51422744859105, 0.902309591192805, -0.0334079189703622, 0.228215111329631, -0.171800484982616, -0.341286497378761, 0.603899127334273, 0.173042634745601, 0.140040587793455, -0.92567276422771, -0.717244323397943, 0.843230961725185, -0.972926516297165, 0.551919561112916, 0.313996195892712, -0.686214026213527, 0.740911310458361, 0.86317435594759, 0.338312530872687, -0.395312584679841, -0.406284335096979, -0.1615953052185, 0.402990178994927, -0.862986626010265, -0.0895448183170162, -0.770836673382172, -0.726141354578148, 0.648431853989407, -0.746182917698654, 0.263872250918141, 0.406739187496002, 0.578020554186455, 0.468717960614672, -0.715410060660102, -0.464000283675032, 0.100036996968125, 0.852966615929868, 0.840224030484842, -0.0728445699568996, -0.72046201620587, -0.336765227463615, -0.197680239555645, -0.671738243046655, 0.770583236026662, -0.441225715380686, 0.92739738221186, 0.68996944628856, 0.269716473610208, -0.116034533456009, -0.489271620174996, -0.513776609950445, 0.95345087524198, -0.272378424633853, 0.868933031791307, 0.913060121842197, 0.595374802004794, 0.310802835097258, -0.101839853862688, -0.851473230819544, -0.0433685285958612, 0.0630373959530071, 0.322468628632788, -0.161753946548653, -0.900788794034023, 0.50044541479636, 0.651704929531663, 0.850721639730291, 0.958895588578963, -0.507109480908482, -0.286262528084682, -0.0931779264213071, -0.729824530121372, -0.598719982459916, 0.425963345525745, -0.588739898315153, 0.911106019499385, 0.931268375708644, 0.545148119397744, -0.513279898326995, 0.933344060879683, -0.917855559569352, -0.850141638884721, -0.79500330631265, 0.221287977749625, 0.141494527878961, 0.279397761274872, -0.836898555507103, -0.959610310339136, 0.2559549880543, -0.34618683741888, -0.262181622725309, -0.891614302938392, -0.680539672911475, -0.557931051450305, 0.428404258179193, -0.356982058600313, -0.712203932070469, -0.712671174253674, -0.469544770259287, -0.246620709076167, -0.117380089358511, -0.978492517467468, 0.917594838689622, -0.138363232580074, 0.320760764718659, 0.75464578467552, 0.320209078746042, 0.0954167366998622, -0.929489448188003, -0.336521710399788, 0.495026701952816, -0.0635891759979177, 0.251993111516959, -0.176516473985673, -0.150083938869595, -0.173104390296243, -0.116982430397461, 0.351796789162088, -0.573887448325563, -0.0150273178029643, 0.921422746383321, 0.638344045365171, 0.297259380690846, 0.131579909593847, -0.316721136623455, 0.786372892018428, -0.625125087691033, -0.931289745506757, 0.572030146665631, 0.758683832265862, -0.34827039408222, 0.506807223082301, 0.918034811530438, 0.1300657965453, 0.655191286132964, -0.792903059032393, -0.827305881983434, 0.841954520827911, 0.0587778307507378, 0.745817393518338, 0.992734772139864, 0.821233271514, 0.953143873194579, -0.946983843185045, -0.963747160445035, -0.905015836952315, 0.26459024074425, 0.744041087085874, 0.25915829547151, -0.9448267435986, 0.911190938403676, 0.163424616173226, 0.340579241437972, 0.442238151960881, 0.188311713610381, 0.860037432522554, 0.441532873350865, 0.814173891710758, 0.799273981048035, -0.307318638250609, 0.583259333759977, -0.497885993592715, 0.356096333835752, -0.915394128310684, 0.563815973985287, -0.971771701076357, -0.756202677353019, -0.113362205053812, 0.47173432100717, -0.240685111299106, -0.243691242353707, -0.490026581432358, 0.471873005801087, 0.430213788120141, -0.101628202967873, 0.370463812864323, 0.359339281533751, 0.237817310344038, -0.0164456925383796, 0.635968085896562, -0.71895773694792, -0.727354608529177, -0.0877723593638282, 0.11233824700319, -0.267837954289697, 0.160381172469767, 0.522775556700992, 0.572509134240685, -0.135473776379655, -0.829874222159267, 0.0362353733770377, 0.42719150080025, 0.874389451456137, 0.312514389052605, -0.494458602742989, 0.494356177081998, -0.427073491484726, 0.47362581243234, 0.232596075333839, -0.364101977897391, 0.540414309546617, 0.57620322539911, -0.0344350548567647, 0.869388503811122, -0.814740510573067, 0.386350908542402, 0.13126918814449, 0.770338687831002, 0.681952651531801, 0.39282456636354, 0.640675201125762, -0.433166638382964, -0.329439125302466, 0.344529687609772, -0.723376261603725, -0.219287966746139, -0.704836027570936, 0.0058753397745761, 0.0953138842021264, 0.746779523315318, -0.329575964435791, 0.000149297502014534, 0.550564496901349, 0.944798295341676, 0.616147415160487, 0.293137697438059, -0.411339732616163, -0.859077229326525, 0.396554270909857, 0.564860379331575, 0.861328727883104, 0.95145400421911, -0.0841309820428755, -0.945663808584624, 0.938998022910037, 0.396880713227054, 0.315211515162899, -0.0116404158774235, -0.0116571749527437, -0.864275813549236, 0.807839164001645, -0.570193553120012, -0.761293994978322, -0.683820885018624, 0.101435451490128, 0.947866516807589, 0.589312133159641, 0.579042771383133, -0.137111833248025, 0.641690238716983, -0.890577635296263, 0.150891886711709, 0.530748109228229, 0.982618682365684, 0.339308188400296, 0.311041652257152, -0.91672508732831, 0.36549434004611, -0.994924944147788, -0.289567687154558, -0.491249915763414, 0.64196223503839, -0.872736174797935, -0.0763666104792935, 0.568446719392922, 0.913805691690508, 0.388059443014482, -0.452460906868349, 0.422460611039619, 0.0788229415547053, 0.107307951397496, -0.636325500227379, 0.0505383966471027, 0.0331047552459345, -0.0389249687179416, 0.862373049455897, -0.425202521151669, 0.339616060485018, 0.214813760575123, -0.964524455408666, -0.427526222404747, 0.174719496888211, -0.101406594276437, 0.388424839507223, 0.235916134834565, -0.963545146348378, -0.0101630042916345, 0.764212326653657, -0.302080127795041, 0.395788184868782, 0.181313200915479, -0.91563977552418, 0.182696332418092, -0.421789232691876, 0.595409216998884, -0.286937541868568, 0.0134913064016169, 0.415125350714056, 0.681097798697952, 0.820086989255948, -0.953845359443761, -0.862695536217832, 0.789962752739213, -0.653596515947654, -0.316926871057617, -0.982784232019027, -0.991316958321684, -0.972984531988308, -0.282838258049434, -0.337661773111923, -0.397923169007313, -0.866615590187579, 0.998323195160978, 0.920286694804919, 0.15505729126987, -0.528916188609608, 0.705767031304061, -0.985822358376213, -0.140558640882607, -0.389544975359517, 0.933519699187718, -0.518419875477013, 0.713616454508741, 0.891969259207254, -0.921352554285083, -0.983450036415709, -0.667944265575914, 0.45505453649067, 0.430662251361646, 0.900449889391487, 0.937550424147144, 0.273964243894924, -0.890266119125195, 0.491016161771972, 0.420165329661672, 0.735832272948179, -0.319999363025263, -0.621459641603965, 0.753286094655443, 0.381165310785641, 0.29020602583872, 0.966018267482937, 0.0953042460932938, 0.669702421740663, 0.421907111982632, 0.242161139258742, 0.780320688742798, -0.346160732380274, -0.055713389888691, -0.773424476294745, 0.4034234845219, -0.844265716145802, -0.592282930112068, -0.589813816401676, -0.143190027993628, -0.182854005134039, -0.714480846379584, -0.739212691570418, 0.918773037099076, -0.640955680169285, -0.114802039101943, 0.285619127136496, 0.691619501099323, -0.396730263928136, 0.342404226367559, -0.429652260121585, -0.334398243973398, -0.365533686966045, 0.0686236463628689, -0.0816705871711578, -0.225689392040194, -0.468178843267218, -0.974473863575725, 0.296282167800534, -0.570160363096738, -0.791346071042514, -0.314586160837474, -0.977788446607266, 0.00450556811031055, -0.414139020549875, -0.732854181069509, -0.505345752714442, 0.706767527333523, -0.78963643599697, 0.761496580446382, 0.958273021772384, 0.343804948065716, -0.930822717953615, -0.723434641693301, 0.0294789892250602, 0.956412878848138, 0.367561769264729, -0.852850146852592, -0.555805292405147, -0.401189099026361, 0.808257072635231, 0.950698696895634, 0.253799980033256, 0.127748070794756, -0.574352492796158, -0.389932633965437, -0.498216718814783, -0.196750493580408, -0.656085439538415, 0.435740280809494, 0.785170705734985, 0.531340222997904, 0.612878122687989, 0.367276553210995, -0.620438868780737, 0.358582303066813, -0.950453972281152, -0.242902544301762, 0.0950138238831866, -0.153585582337513, 0.331179782154381, -0.00452596291060181, 0.895810020108873, -0.266719397177937, -0.706131205535079, -0.328484920748898, -0.65082493748737, 0.1397462719696, -0.145233888882763, 0.466848317641725, -0.119164421857127, 0.0636133535776158, 0.526527330016086, 0.227331992114208, -0.407985692412676, -0.646694548252196, 0.666502094901734, 0.976848816676179, 0.780096433405346, 0.18366212733191, 0.048491928866659, -0.2537264948752, 0.609152938052422, -0.973329856796432, 0.640917884012351, -0.219556148671579, -0.542705172463279, -0.807798178334608, 0.249292715257738, 0.412939468479398, -0.702064920373311, -0.585646927123626, 0.268439864873725, 0.840582355559886, 0.879022308995711, 0.00394581016581497, -0.932272170359456, -0.644282613133943, 0.983491948878434, -0.841150673293486, -0.48203500977043, -0.755722289828345, 0.255495573699314, -0.179543253713438, -0.267291790121695, -0.862114520735545, 0.0628825724404793, 0.509006525601562, 0.366565251723791, -0.834975439623199, -0.511764173561789, -0.042727807707567, -0.0299260396057071, -0.00721880860343238, 0.320136319527112, -0.495865368589875, 0.797328810757591, 0.792948416865918, -0.402719254132024, -0.446009234742298, 0.711831326706027, 0.0656772660594083, -0.500579354934355, 0.796681676393254, -0.555099794014813, 0.61171454966593, -0.428471863037259, -0.0111447795285855, -0.924799291711653, -0.447627603863755, -0.720037702001412, 0.38791539451504, 0.990260240219465, 0.438354241988205, 0.101264597529538, 0.929752204207537, -0.677094491693244, 0.540210891170467, 0.279767505455907, -0.83578024820849, 0.0335763025594207, -0.522602427015701, 0.0432192662705659, -0.977033170645511, -0.852744371100876, -0.320434555215638, -0.499942848608395, -0.51219597588936, 0.690839342732928, 0.673895442355497, -0.284084619511289, 0.883316978581175, -0.939944555241099, 0.104765954863113, 0.737481671928325, -0.830636708638945, 0.917269851468797, -0.760076597576049, 0.0124880016896376, -0.911671329284759, 0.0417644990983979, 0.352868233944631, 0.979997880194731, 0.3714307050279, -0.934247265636905, -0.965865152064709, 0.0509084008514973, 0.443706271989914, -0.702343180093626, 0.823205205086753, -0.816412702307295, 0.297653656233522, -0.172800977571564, -0.233417433186337, -0.974547149759002, 0.118592731486748, -0.825117857497014, -0.0254193392077466, 0.186719993044959, 0.0701693519955535, -0.649926419119438, 0.726172146438437, 0.716870034736603, -0.116249324719861, -0.544706352103665, 0.691449341002427, 0.084276948684042, 0.083641910836773, 0.59660755298164, 0.720778980456965, -0.255881003832362, -0.653217387734049, 0.206797216209331, -0.643483757865776, 0.457527591051805, -0.641756123597058, -0.689930638455031, 0.196584653328557, 0.138330888521439, 0.47969976015528, -0.499689205317243, 0.61058330634273, -0.936749276085433, -0.390231711894548, 0.239720369913488, 0.472576462869394, -0.325548267349421, 0.458518412508969, 0.880465877109145, 0.668842306787564, -0.335509867214529, 0.24920172667637, 0.888536126127232, -0.502752947439099, -0.550553268241209, 0.123701639519744, 0.191141170905408, -0.506703355573823, -0.798282859303291, 0.789582565873234, -0.790518365898006, 0.0549697466445807, 0.208337258910324, -0.803270640767937, 0.197545561932698, 0.731923858551825, 0.739383649586997, 0.871466025783734, 0.258058436642594, -0.099921648921665, 0.448533825314956, -0.662579510679888, 0.830063888500342, 0.0067007565558197, 0.941663715390075, 0.388146675166007, 0.311944592756839, -0.34685948319441, -0.697356568720568, -0.837215994366721, 0.772383844322792, -0.0840437786130077, -0.191528918493272, -0.736851772818252, -0.279730113429524, 0.36591748990769, -0.933406890881219, 0.180531741646136, -0.781613199711042, 0.150434789891922, -0.889694161857798, 0.79713935680129, 0.13081434346298, 0.402463917287476, 0.239345615317774, 0.933077159482232, -0.158387892512693, 0.65675317998606, -0.47729408207512, 0.380099910150873, 0.157043830254219, -0.239851018018663, -0.514686881541922, 0.37340935983698, 0.822376601987556, -0.542228048867322, -0.322568091086268, -0.652495864532796, -0.966688140888245, 0.23300363247026, 0.33554215772362, 0.24314201669273, 0.502441403358778, 0.399583196318354, -0.820028702128187, 0.730148105475839, 0.499950934706524, 0.761208484907288, 0.751200734576505, -0.620872811301528, -0.0330812545674877, -0.637156897309476, 0.591308473733391, -0.372931544178246, 0.812939005736506, -0.509432714512422, -0.898072971942388, -0.367510947735212, -0.345851771997683, -0.40046248055765, 0.622659006477577, -0.00160938894123897, -0.844762395623476, -0.38973202398284, 0.177189717656645, -0.866609206295877, 0.834822032547886, 0.996408040420876, -0.992687161080956, -0.319583010738675, 0.279053727528446, -0.451143623498683, -0.377300581601586, -0.403178337224207, 0.426987675822103, -0.588971328583494, 0.559454514447605, 0.579208142751724, 0.488607289980226, -0.339440064225206, -0.340958140525993, 0.756474428628564, 0.582065437798527, 0.847295146440445, 0.7813464924596, 0.114523466228231, -0.85228848640619, -0.395334129302275, -0.147419455003545, 0.860238785129988, 0.167807880052843, 0.635851600246803, 0.410136556910752, -0.70880586149883, 0.15784875216864, -0.343796721215953, 0.0427680992554378, -0.834969567125379, -0.52938739694867, 0.54963838118124, 0.16977535697214, -0.49585770246692, -0.0359041046412505, -0.0177378126932193, -0.154249186913731, 0.378136670485831, 0.577310493964542, 0.858453026945062, -0.690854896282005, -0.605599212806222, -0.711009355131341, -0.783518005478787, -0.360931208680278, 0.898211260196749, -0.148506259321103, 0.582393609633846, -0.60990423843571, 0.0141722234637072, 0.562535665551474, 0.808698890831441, -0.507916912486155, 0.247146112427222, 0.0389678566340821, 0.275105207471293, -0.204877588545981, -0.692915625015524, 0.924954321616893, 0.459706035488494, 0.840750404253418, -0.442916007113308, -0.62155314320188, -0.0341242918785789, -0.315765956511139, 0.236707610360701, -0.704037114550553, -0.899295253699979, 0.23133315954904, 0.943492897241303, 0.346281132204269, -0.68429722722727, 0.761138084137817, 0.909622801393288, 0.726096675634902, 0.517156376981116, -0.485737056766012, 0.0143351249127155, -0.269702121478406, 0.96679419271649, -0.500309342364955, -0.968592997498537, 0.846281297366823, -0.173606719030293, 0.178098007562956, 0.455823401599917, 0.740440471780683, -0.20145193570428, 0.877255961244934, -0.59502319263218, 0.893816004551134, -0.169558231210595, 0.357619090613509, -0.660369925652755, -0.367603738824171, -0.760434358744435, 0.106489446989681, 0.492811067457645, 0.192201787031896, 0.194562925756296, -0.658066363982491, -0.578303642125385, 0.571231576463543, 0.277867422717252, 0.553167363741839, -0.332116299705967, -0.907462061512416, 0.328058350443847, 0.894641999296504, -0.262076401556888, 0.626196838323197, 0.83680219007549, 0.54551827911067, 0.260488988824374, 0.369263528702509, -0.727427210767865, -0.95252841175562, 0.113863899451655, 0.319161413306613, 0.727442925650574, 0.413942696172905, -0.770990053437207, -0.0436876288328452, 0.00368799764738625, -0.649669742231628, 0.347539153288246, -0.45815859072865, -0.103100885620927, 0.849074327153537, -0.644378862364136, 0.316982886799805, 0.444216603900785, -0.18198182205819, -0.2010655912393, 0.0205659254561044, -0.868708262569108, 0.794636395161449, -0.993750951643801, 0.00789033994959243, 0.843633353950545, -0.450066687917481, -0.777652934728258, -0.450741973529013, -0.934420937176596, -0.42410836552627, 0.290531450166697, 0.933593902233363, 0.227468935279755, -0.544346198156388, 0.258253335717119, 0.196705397276155, 0.57658989543007, 0.0978309707919365, 0.882835766416455, 0.545674896366948, -0.0492914484537292, -0.896874443606977, -0.800429233209853, 0.623478543306917, 0.418269576796554, -0.518635502080004, 0.215676947500029, -0.38915959567233, -0.056870086051275, -0.700314820614243, 0.324458504107461, -0.923588368477893, 0.998569129477957, 0.188623437147476, -0.962958558003246, 0.557944806258205, 0.981762179434063, 0.666352285308778, 0.835599697030747, -0.642077466161336, 0.499010283248603, -0.537637387436834, -0.477854785106394, 0.526755337903742, -0.340675117339844, 0.906077652288505, 0.800828794371298, 0.27081170473972, 0.602378772324124, -0.0364836310682701, 0.479521180794679, 0.737226136595567, 0.653277308536679, 0.416756516243422, -0.147677478980729, -0.536361196813697, -0.66137643290338, -0.484322437740776, 0.858956694939688, 0.572881472429371, -0.834114165953628, 0.302078088010444, -0.244659112413534, -0.787415151825225, -0.0119354166983774, -0.485836728433656, 0.282490122886245, -0.104802288246445, -0.584472185936006, -0.888925410679024, 0.936047646725657, 0.463781240585344, -0.886531105864137, 0.939022087631474, -0.695589656556905, 0.34293040218017, 0.976103881225487, 0.781446617073731, -0.126689512260607, 0.434727224686929, 0.804666466911494, 0.963983672198848, -0.575161851878849, -0.908753582385938, 0.205013810832256, -0.834077477210591, 0.210300222273193, 0.910769683315382, -0.411296705109494, 0.952088321073667, -0.493452964303271, 0.584833263098778, 0.71288376382129, 0.00633457909253443, -0.924454141990519, 0.71656704726721, 0.0188748421225404, -0.584163317534614, 0.876190295010479, 0.811912967501626, 0.39458025344619, -0.144623621258772, -0.580536706576638, 0.343101594689518, 0.372958629188487, 0.301094452098605, 0.736432464955091, -0.63760044617719, 0.735809025769448, -0.463548425806364, -0.654026588401289, -0.848342991382925, -0.978276293224553, 0.727951351200526, 0.437726084978927, -0.0579271926610241, 0.167990143392625, -0.838707212186224, 0.16059521613511, -0.159461862715589, -0.572388369192068, -0.902105316582251, -0.37635054281053, 0.0612967801494604, -0.524704474916942, -0.328436717907477, -0.421437373767437, 0.241946499922072, 0.342992246478169, -0.950081910304674, -0.0460333899100211, -0.194300682642229, 0.0579280114412721, -0.353048444609129, -0.68041942414171, 0.353225638387772, -0.348656221582567, 0.961027385126682, 0.425148059984025, 0.1656312172595, 0.259115820790054, 0.321725369333615, -0.318419342983654, 0.128713641639037, -0.439534416715787, -0.237806648017784, 0.321637955944649, 0.305009560415617, 0.404970775561725, 0.380993556947555, -0.155658518475856, -0.46766917088556, 0.701437091585871, -0.514891284717674, 0.737191354269839, -0.56726643812615, -0.533716660831212, 0.934756935605172, 0.312610302991813, -0.896392391495818, 0.0588179908693389, 0.427432516078639, -0.107535980588217, 0.225967555639908, 0.894380235498835, -0.838222854632189, -0.273615818671876, -0.931255877247975, -0.798449077944885, 0.976718768224181, 0.681044076037217, 0.621078414811558, 0.41952776971363, -0.711484648454819, 0.275123930348345, 0.0330581212104255, 0.701582888538026};
    cv::Mat bMat = cv::Mat(7, 150, CV_64F, datab);
    double databeta[151*7]={-1.36890066350853, 0.630140980548511, 2.66075669335836, -0.673357881396975, 0.811806861732062, -1.16572315971939, 1.20779431418898, 1.22173960425319, 0.564002632154476, -2.06811747388949, 4.24718600963931, -2.52720730032774, 0.472427631481805, -2.02768968983168, 0.947713025573069, -0.0686703993525818, 2.18017261737337, 1.12842810782752, 0.422982136475048, 0.02868736227555, -0.751222958025031, -0.381761380265513, 2.53841384596119, 2.38470254886463, -2.25321738370111, 4.02614366926622, -2.51681628134504, -1.0469361818838, 3.42086846513403, 1.37347241857699, 0.30277375634983, 0.314455510849581, -0.143298656195844, -0.143232721852731, -1.43809343697136, -1.2076697648557, 1.52832184297972, -1.48681343929286, -0.771054086538083, -2.94748937639567, -1.09066228079123, -1.44433153339107, 0.655080732667904, 1.36839995023104, -1.67596939797137, -0.724147604429945, 1.10803077213393, -0.20651861259123, -0.321467257006693, -1.57843626894641, -0.373733770683109, -1.55879701614782, -3.9229809534757, -2.6003785436074, 0.331242887277898, -2.79435413529232, 1.91955311039482, 0.189418244239403, 1.90450285076655, -0.219007747508214, 0.81429621375906, -1.49117111173843, 0.182290267924087, -0.685293777120787, -4.19467660016315, -1.92734116269438, -1.69878623251435, 2.06527951242962, 2.59612158479417, 1.83319889445475, 2.14594922767355, -0.862853865827306, 1.31492001505581, -0.236246258147667, 0.43216588333841, -0.0853597512357174, 4.06426369902725, -1.72811720159805, 0.0942605605947444, 0.776232850929402, 1.03430623318345, 2.07424666823387, -0.780486365821137, 1.55768339352005, 0.934079696880522, 0.522945409908826, 0.273917841942905, 1.83166853351656, 0.128935995192413, -1.04278757946696, 0.350810980979857, -3.20474060120862, 0.859866970854844, -4.99917088364607, -7.25574858074546, -0.942580243598333, 0.17850844511074, -5.89273883068277, 0.923527471249313, -2.60264195541988, -0.576227685581027, -1.9944364011402, -1.64150841269778, 2.09169095119454, 1.82039146769281, 1.05810464158697, -2.28196544082924, 0.163786853539762, 1.57453814785608, -0.323294077054256, 3.31486881270009, -0.397048630284208, -1.97350561034575, 1.49523654685981, 1.21394552383877, 4.94941201519818, -0.810179457321839, -1.15643511763382, -1.93458535418355, -3.03769620192456, -1.85652691037991, -1.70738132139453, 3.53343732967783, 1.30768822117681, 1.83511800286439, 1.03326747528151, -1.4507447131063, -1.1718167349816, -0.211067681271209, -1.43159441377631, -0.206460555918893, 1.05185293797898, -0.650730711972322, 3.2641352987911, 3.27407215819469, 2.7477968860184, 2.90046213913393, 4.08333048941088, -2.84207450299788, 2.49656028962119, -0.848038901284793, -1.59274153358023, -1.03152930261483, -2.59884351368843, 0.952331614873342, 1.38242683371531, -2.82691693909786, 1.07104653264334, -1.22022756171605, 1.46908321612927, -0.521971640078363, 1.325540253694, 0.742945781587314, -0.706163542816198, 1.31944876019883, -0.123364851147428, -3.08016261512435, -0.0320071950001235, 0.695800218305027, -0.164295059710419, 2.64846231276438, 3.74256496835639, 1.79252875101717, -8.31807188633657, -1.82543306334825, -7.38068912579253, -0.124950644228376, -6.77839837297939, 2.52163555142562, 1.31582577989233, 0.588287650818079, 1.56151667271622, 1.19290027161239, -0.110377647837821, -2.01981967706482, -1.09290944674844, -0.665668471348914, -2.46672127287742, -2.78533175494955, -1.12156634579732, 0.570381819555003, -0.163660992015081, -2.76007643346915, 0.939348691562869, 5.06342684303668, 5.17356171133366, 2.0189497700051, -2.26421409869032, 0.563935221092727, 1.81135670076887, -0.180703337346301, -0.0301564480461179, -2.18057326887313, -0.0589555633192051, 0.472846943259959, -0.673319689976206, 1.04231652659963, -1.93454126392634, 2.40258085599212, -0.719312569678577, 0.0100167953036763, 2.6837050898643, -1.76401004810468, 2.88013579129612, -1.03478384625214, -0.506678526941707, 0.250907380267486, -0.90628729276355, 2.77808065945507, -2.64245187297693, -0.756408215267798, 0.163746252307581, -1.06463675170903, -1.02084143068122, -3.11946851706546, 0.0944709365872743, 0.850937154022345, -0.278635847407349, 0.686439320067167, 2.39614998058216, 2.36294892857983, 1.88649744264614, -0.630532736544058, 0.0619048662636331, -1.96280974321226, 2.35567362396564, -8.23197504328843, -4.68619661199495, -4.84480715246833, -0.873121054178132, 2.53076714308504, 1.22296423387164, 0.651098944338867, 1.73063494271627, -3.26502061974296, 1.71796987307591, -3.64691864602633, 2.69948174592607, -0.620348004378411, -2.58540074702452, -6.78317125673808, -0.801607061396411, -0.526177948058762, 0.0631370752192767, 0.929864517652442, 1.31840242013775, 1.43584961793056, 0.967657571870363, -0.487579127749526, 0.509689232677125, -2.57177126276349, 1.49130266314552, 1.95267491745742, -1.65783546651666, 0.631953108578812, -0.0274892793106693, 0.318619977638765, 0.233782259161704, 0.841171170101732, 1.80236596893654, 1.85872593469589, -3.71079271399267, -2.48307727103803, 1.77682098048005, -2.92642832257466, -2.7205835241944, -0.252521765062547, 2.88742621097529, 1.16786785094262, 2.84225162794367, 0.0899780659868911, -1.5526476859069, -0.939788121109115, 1.24324312755489, -1.96849109712532, -3.06485530899389, -1.59115252167436, -1.13030137102512, 2.48938482879851, -2.67431070393662, 2.73207346765228, 0.6441009649819, -1.6109370970129, 3.09242278079087, 6.10812292393688, 0.280848866330021, 0.524313338588059, -2.67293456203192, -0.602625079863492, 5.4939291942469, 2.2526151528678, 3.50778514793785, -0.157319208655891, 0.213724524494195, -0.102472226474688, -2.06787635433586, -0.847246135275459, -3.70950286457688, -0.561813028897821, -0.00523156224129651, 3.7565301484499, 0.379012083481426, 0.740215012176387, -0.687303462266474, 0.364770914015634, 0.720477269812142, -2.42198079080428, -0.841430839096406, -1.29419400135674, -0.986970583732298, 13.9832215176912, 5.61413886961971, 7.96792954830694, -3.66887038638858, -2.01100405429578, -1.67795189575884, 1.22754402905816, 1.39299440719001, 1.87169893261357, -3.8291661104218, -0.413758914115126, -0.574820667528877, -1.64493956537528, -0.529202136555566, -1.54612347555479, 0.565903859777183, -1.17153129077998, 0.714881059668983, -0.348317128916691, 3.28782923120152, -6.13084238860218, 2.17207334584905, 7.02750538281864, 4.00975438293184, 5.69415826561928, 2.08167586279812, 1.17617494540118, -1.27564847990055, 0.0359651643891269, 0.987387615419198, -2.32440174309327, 1.48159654908271, 2.6387197772422, 2.37558462740121, -0.122511018775855, 2.38114383253748, -1.06642966688089, -0.722248591035102, 0.0156545614150138, 1.54690557700175, 0.576014710584104, -3.55461126140058, -3.79996829863857, 0.505700277482569, -2.15176210617145, 3.17681925154085, 0.237210363300272, -4.72796886590725, 0.436175115119524, 1.19900226378808, -0.997968262532685, 1.95574111860785, 0.394962133527124, 1.23118077833681, -0.832285222111, 1.70688224385906, -1.20449472470535, 1.37233831572871, -0.53887731028713, -1.60357727170296, -0.616290412769923, -2.68895372384236, -4.55869344504115, -2.06105757747793, 1.58207123858172, -4.21095471453989, 0.426293108653363, 2.00656403642237, 2.1655482324247, -0.246355734218229, 0.564827140360086, 2.74714356331045, -1.44678221748199, 0.282003841209064, -0.143969045814095, -5.76236442814273, 0.701833798405526, 6.43739617650319, 4.66677065698208, 5.47158500683558, -0.558680439361883, -0.521333030883616, -1.87566793171807, 2.66306259610264, -7.60567392823151, -3.94919619756702, -4.39738210756705, -0.270476873072534, -2.383720079192, 2.89758574492512, 3.17043935121235, 0.553110952395681, -6.12853951862165, 0.81301320658309, -0.866765250593997, 0.780646730618777, -5.2417665817142, -1.16837700088911, 5.61540976204715, 5.658293546166, 4.84161155688477, 6.65694796204518, -5.77802467793243, -3.01958117819317, 2.33905615109112, 5.59386450330537, 0.502948199064613, 6.04121733066537, 0.361625686934518, -2.50824839973934, -1.14171756807311, -2.30695635794776, -0.644324785675757, -1.75769629915126, 0.215647128709093, 2.83709273281295, 2.35933886655588, -0.92904327103856, -2.05787044573363, 3.59259903693723, 0.391202079693496, 6.34014400112018, 5.41867012305877, -2.76472885858903, -1.34940460550582, 5.90381682273447, 1.54716830991686, 2.71831921482836, -2.25274320266672, -8.07168065792454, 3.7153680965267, 1.28173945381342, -3.22506982648832, -0.554811803576614, -2.74343410039711, -0.155723698615567, 2.53417166663414, 0.259680775253867, -0.222616068963879, 4.16975283051938, -1.88827749385912, -2.73896163005632, -1.5700241650766, -3.59081166204824, -1.01177405177321, -0.0549015088729321, 0.412366912135462, -0.66864525357969, 3.44024899598727, -0.539603640407977, 0.828018354251947, -1.49301821959261, -0.045602005813292, -0.537116267329565, 1.92388444804316, -0.349338594110495, 0.81011901083966, -1.33315455981525, 0.662703148945014, -1.47960749551545, 1.07771640890528, -0.0470009739949909, -0.537994342662389, -0.09994030082247, -0.846765491407953, -0.252124928579437, 0.496787325229367, -2.7913123068344, 1.6187874049825, -6.07212659081192, 2.38332280373955, 2.55672236297984, 2.0455473233053, -1.1448113909984, 0.515407298117641, -2.58050870338572, -0.404376369685214, -2.03101174250108, -1.66008919704137, 0.61387464397279, -1.07273358902812, 1.42725898353145, 0.8521111859443, 1.5341691484965, 0.467965324478399, -1.69455965384746, 0.146998325148238, 0.029699176511578, -2.24111468522313, 1.39155513290257, -1.99660275188498, 1.49446733878161, 1.12988623508594, 1.15271013032611, -1.23912138789327, 0.479386489974717, 1.41712589960213, -1.13048061092407, -0.737307975734187, -1.18527772288459, 0.916125341329681, 0.84457054690442, -0.50406134818887, 5.25308140548558, -1.26873180353594, 5.18937980236167, 0.215526594465468, 0.752727310302105, 0.961988378828894, -0.06456294192868, 0.606129170441339, 1.36120572136357, 0.771081102407779, 0.0683247561016088, 1.19382335378992, 3.36575997784657, -4.31501254259833, 1.99825594680588, 3.49498755039762, 4.57732799335709, 5.09179094688303, -0.198692689804673, 0.334483025503299, 1.42932588252348, 1.37634606610427, 2.05576399483006, -0.596957045559616, -1.59341231947853, 0.783354599746659, -0.889852132378627, 1.31983946188578, -1.19820155420812, -0.207009353015275, 0.105727368707565, -1.34423671245811, -1.75555004950344, 2.42778719385559, -0.840828489633213, 0.469488415253918, 0.980983430650479, 2.33553988584508, 2.07071576964687, -0.682087762989651, 0.512284636637264, -4.86506749637106, 1.13277313877801, -3.73307636438144, 1.06991815133976, 3.01397189513344, 5.18995092555573, 0.816568832068583, -1.25476290483826, 2.01598142154454, 1.28310376780684, 3.99241543225927, 0.672929680747229, -0.761692512778092, 0.8953748744386, -3.47182883717087, 2.02790167291957, -0.656155898227667, 5.54673251046696, 4.71283437371931, -0.616591809825406, -0.430550258019656, 3.54008915733141, 14.8104657542855, 2.36531914740679, -1.69840605729229, -6.86247893730328, 1.73193232057042, -2.99177706313651, -2.91635756780112, -1.0164234432549, -1.47101046926843, -3.90860377128346, 2.51240214156291, 1.29354604767977, 0.476621193729162, -0.93123333805654, 2.95989755408774, 4.42479005616128, -0.0224674023320316, 0.253686126133797, 0.478836598527088, 0.0460217910757312, 0.188657539078158, -1.36380877902738, -3.25867607519557, -2.53061081512784, -2.08547834667306, 2.86449630795882, 2.16912053971062, -0.982150277399164, 2.57100176972093, 0.0365670182187754, 3.97067343052759, 0.832596067689862, -0.99935037223493, 0.53511627249023, 1.56908468369777, 2.27573467906668, 1.33877206951634, 4.30406473050433, -2.54068991782633, -0.324394178231417, -3.01097441775625, 3.24423315682521, 0.597655640639195, -4.68267902653326, 0.475358365096713, -1.49219453820756, -1.1965419574212, 3.92759218940304, 3.21735118812444, -6.44961440335838, 3.72130553511832, -8.41037558651431, -5.138715470098, 3.26447083203419, 0.922643915834243, -0.537155902412329, -1.73390197248212, -2.26509718011885, -1.66009487093433, 0.426944694360977, 1.20881855061089, -0.44893959594961, -0.600738340746612, 0.421735515069717, -2.48326782524215, 0.0972677332289216, -0.132013359925622, 1.04453163940362, -2.2852245239593, -0.677691810119738, -1.52446752399505, -2.151639616062, 1.15246443329922, 2.0489560951231, -2.67436233509628, -7.00199781554778, -8.27252286211727, -2.74907255771396, -6.82222352022743, -7.45881015832705, -1.43716238325518, 13.2088444862069, 1.51211779384808, -0.426270004785248, 2.21405039966442, 0.684303375689579, 3.23683046722293, 1.80928190939179, -1.40530401879954, -2.13213854023834, -2.55954195177009, 0.77992103620835, -1.81861811394096, 1.50795761198415, 1.33644968491031, 1.49583245585583, -0.846653875657139, 0.948043344671334, -1.74217955878512, -3.14890736127804, -2.1636121228548, -2.12108840161127, -0.181166292587719, 1.62702682366853, 0.397933804505046, 0.935824685489847, -1.04200532156881, 1.64596153912015, -0.521215480795539, -2.66069823115442, -3.57831848730014, 3.22653924893052, -5.35975648739245, -5.70273501568345, -2.37917935192358, -1.90892545991869, -0.908269937993157, -1.75505599089353, -0.67290689097715, -2.87124569218941, -3.51508054172319, 2.46304870358102, -0.510824603990147, 5.25618307649316, 1.62364914764805, 1.23556268666676, 2.09492070905282, -1.42639450650357, -1.84978418235475, -1.57903079448414, 0.357669304452697, 2.34289487118573, 1.20829950220131, -1.85770284060269, -1.06817357573965, -3.41600146618294, -1.79631911356956, -2.61849701157859, -0.206740939623622, 0.341882247899581, -0.152276636001514, -4.09418144688813, 0.269467459318943, 0.776500879676773, -1.4157317869778, 8.64599818228991, 9.12698667876545, -0.80569380619007, -1.84154406190965, 2.89374128440125, -5.44151674730047, -6.28908624656102, 3.58471371245281, -0.525141832272467, -1.78920195541372, -5.91304222800144, -1.54258980493534, -0.462689521935764, 6.54306221067775, 0.615259854322628, -2.47112062082643, 2.02564515584924, -1.45288316964458, 3.00946255985362, 1.85991214916867, 4.35216712530721, 5.19866248219312, -0.550738484103057, -1.91124190902822, -2.99017316551436, 2.03489984688073, -1.41069560292142, -1.7031253725604, -0.464495381218984, 1.05500462703597, 1.3416977125262, 1.94139573426307, 0.529065049949685, -1.93356673575182, 0.0086869954531389, -1.78260495403399, -1.19753031987303, 1.86305657915075, -1.820418170188, 3.42929125472052, 1.27102489096975, -1.84163007882394, -0.0837646373783408, 2.48406076780974, 0.685909546079087, -1.95827605848426, 3.23756761797443, -2.84801784364885, 2.65687214605252, -1.59405005389332, -2.13806828486906, 1.28399793601242, -1.86351349018869, 0.766875461830418, 1.74573593662636, 1.06054080360549, 0.738984688697466, 3.99404208853022, -5.26968910987696, 0.126892500357441, -5.24015143370199, -0.572377189272524, 0.0722191473993546, 5.42377508851745, -3.54170072643959, -1.28974283500597, 1.4630456740687, 1.04036953984029, -0.380321926265897, 1.80608561743078, 1.62551851146643, -0.951467611808199, 1.30711901498933, 1.68678294026282, 3.27347416873159, 1.40706555033055, -0.613369932464774, -1.76557449012195, 0.485915892833026, 4.50117023561323, 2.56207660267946, 3.04757149278805, -0.449219067237221, 0.335714806484238, 2.12157277355503, -0.319162891297495, -0.123509745490031, -0.743943348253718, 0.537192124396647, 0.0615128313792586, 0.233936669216747, -0.344264526167439, -0.0353485706663941, -2.96314317911037, 2.82227633027578, -3.69364673802202, 0.274151346723343, 1.37727707047158, -1.87411959780923, -0.619247792860529, -0.872600765704558, 0.456448509046588, 1.97114069407966, 1.17769535693855, -2.3156356154478, -0.55290011203096, 0.474060673133613, -1.84912124015909, -0.545845427084177, -0.486063041654511, 0.0821496571155267, -1.42712857970873, 7.46462924897387, 5.54104573587831, 2.92201341560286, -1.00379601733283, -0.0136584223715678, -3.44462901645082, -0.497673762463123, -1.21604935586273, 2.67782517287984, 1.22549647960819, -0.744015679303693, -1.43784081529168, -2.77306948405404, -2.06656501116847, -1.64383778899289, -0.155452025790068, -3.48554388545173, -2.32099415747751, 0.0109051788517531, -0.592825077532123, -2.15684200755836, 3.30847779591618, -1.11873984709902, 0.247291817703442, 0.298851211078867, 1.56666528235804, 1.04579400387028, 1.00820780223688, 1.66838885488407, 1.76110386792619, 3.36769367629837, 3.84901111642082, 3.69003048099864, -0.831227130655769, -4.01008900561572, -1.23103415822908, 0.847959718393063, -1.33568655430268, -1.81171668334655, 1.58159043060089, -0.79327373971342, -1.54227823894457, 1.12229191363859, -1.890795617563, -1.08328287059551, 1.32397497599118, 0.0314099545250465, 1.53447098180333, 4.19909122658586, 0.0753232270004305, -0.905984919182578, -2.1904127496046, -2.21132208796828, -2.98895914722537, -0.0429845006526399, 1.43905846925378, 1.87549815969939, -1.81773936464765, -0.208832718129979, 0.79699687553906, -1.93985214073492, 1.83208960379894, -1.30684318076123, 2.1508481677655, -1.19866712466901, -6.40319181880576, 4.1481273554375, 1.60897227804748, 4.69718220940039, -2.48044411293016, 2.00693447822554, 0.640460166377637, 0.786361169449166, 0.18729690667737, 0.86120122108378, -0.0298432061634258, 1.39773018133957, 0.180543615501577, 0.143866156332956, 1.87296540170213, 1.285988789557, -1.12774301196677, 2.52129255455799, -1.13423932757979, -2.38527036705963, 1.55143296192268, 0.141850626778979, 1.29645648373058, -1.4375881151638, -1.93650370223702, 3.10504370368892, 1.65720311867626, -2.50624996836868, -1.97934752984967, -1.24452277272448, -1.39245510027351, -0.296582757106236, 5.5363599573048, -1.5844575067883, -1.60060734216327, 3.20934759335828, -1.57036267637733, 7.77053349048306, 2.52462769235075, 2.06077118887506, -2.54462081824441, -1.40062000905663, -3.72263337819325, 3.69642829338182, -1.06581832691406, 2.40196552670538, -0.106221242908887, -2.93004720531175, 6.52866164506502, -0.663478289990267, 7.79775101912141, 3.48348679219243, -4.99483038426301, -3.50707734758336, 1.30884736379405, 0.243497806898635, 1.55509861094831, 0.511153762299141, 1.6440425289314, 1.05990715481511, -0.0790099727012997, 1.13467132558663, -2.48878216352148, -0.113333597621271, -1.09859553830423, -0.784156017152609, 1.82006816793753, 1.33931848517643, 1.83900360716074, -0.0355036959042937, 0.494244873614034, 2.13752764983505, 1.10003057924448, 0.240684187354134, 0.286495563804813, -2.80270461627782, -2.01401548270742, -2.45973549920499, 2.14014556791376, -2.7000254233185, 3.02798471797715, -0.737756970612277, -0.119303215835999, 2.02054714960646, -1.94665214128701, 0.460598554214487, -4.02467751291608, 0.256221739548153, -4.14425310195752, 1.8165269034871, 0.129345812041063, -2.53782153938417, 2.27375676256962, -2.89786854790509, -1.02284332433385, 0.0688001509671824, -1.32789761668538, -0.803544989904739, -2.70623207702, 0.953071377525121, -4.56031424278277, 0.643201439067982, -0.364291783028603, -0.951581245746983, 1.81348018287008, -0.390564408320865, -0.101774357629494, -0.590885000660968, 0.219527365821438, -3.02595565302295, 0.75188857361301, 2.21347019963858, -0.272148983848239, 1.84653673845099, -1.95822626944016, -0.604490286547582, -0.367543074587522, 0.87983161558438, 0.707276709188626, -1.0834048168376, -0.0842850344693477, 0.652141253816634, -0.439578736307278, -2.76811898865048, 4.94434693869662, -0.373868790707877, 0.926817835388816, 1.99547658717391, 0.478482012379902, 0.218673576113216, -2.30331742408247, -0.744916581889021, 5.23080357899574, 1.41537498718727, -4.85622927319017, -3.36061303658146, -1.57356339489045, -3.63774090179434, -0.906906124209681, 0.63051442582673, -1.81772432531459, -3.31537051713272, 2.02119900969498, 0.0765650182083749, -3.73565468567369, -0.493251403621744, 0.239046647992727, 2.06272938292694, -2.03908376304694, 0.586525729375901, -0.0508267505338924, 2.85404998695903, -11.92619293309, -3.67902247774775, -3.05220373315359, 11.8769155391858, -18.8512993069938, 4.31907015677907, -0.625540515643535};
    cv::Mat betaMat = cv::Mat(151, 7, CV_64F, databeta);
    cv::Mat inputMat=cv::Mat::ones(1, 7, CV_64F);

    //Scale down
    for (int i=0; i<7; i++)
        inputMat.at<double>(i)=(input[i]-mean_in[i])/std_in[i];

    cv::Mat H = inputMat*bMat;
    for (int i=0; i<150; i++)
        H.at<double>(i)=1/(1+std::exp(-H.at<double>(i)));

    cv::Mat outMat = cv::Mat::ones(1, 151, CV_64F);
    for (int i=0; i<150; i++)
        outMat.at<double>(i)=H.at<double>(i);
    outMat=outMat*betaMat;

    //Rescale up
    for (int i=0; i<7; i++)
        outMat.at<double>(i) = outMat.at<double>(i)*std_out[i]+mean_out[i];

    for (int i=0; i<7; i++)
        output.push_back(outMat.at<double>(i));

    return output;
}

void Server::inverseKinematics(double* input, double* output){
    //std::vector<double> output;

    double mean_in[7] = {0.9771, 0.0202, -0.0110, -0.0029, -0.0007, -0.0043, 0.0684},
        std_in[7] = {0.0247, 0.1528, 0.1432, 0.0179, 0.0156, 0.0203, 0.0229},
        mean_out[6] = {35.7254, 76.8555, 33.6986, 71.8366, 34.5330, 74.3632},
        std_out[6] = {18.7594, 30.6429, 18.2102, 30.9079, 18.9467, 31.4114};
    double datab[8*150]={0.560871352321735,-0.0778789530809252,-0.985510913908158,-0.707076627763841,0.956198862729863,-0.324056070605093,-0.0920251650706916,-0.855943597549,-0.052294532309993,0.74383629530649,-0.0349103368768551,0.829327359829702,0.263195691464499,0.876827439512788,0.132996828088746,-0.0717212619393133,-0.0116786332644132,0.193289597535121,0.143290302308517,0.208872379259677,0.686677708629346,-0.826805170953809,-0.233324016493255,0.190442121080261,0.583198754923959,0.79955641603872,-0.64150188388613,0.751949109080942,-0.28310220404082,0.337342521265933,-0.552780085403374,-0.658486925459302,0.179314144172442,-0.701806589174043,-0.305586648361633,-0.358983130272652,0.367483360900307,0.890998871395351,-0.640609683414697,-0.634880812108848,0.328595522503075,0.574002931381675,-0.748877723290079,0.276422053013846,0.68187551123281,-0.792803476375903,0.341289959780578,-0.955265385606623,-0.856926733344735,-0.709106102461665,-0.472380793131992,0.593194975052622,-0.172020484024935,0.802022345875295,0.202593137783121,-0.777286418241706,-0.634892200945831,-0.0899383902580968,-0.278826942666868,0.538560773166526,0.847093948490028,0.523974704974672,0.815406031093018,-0.318954520009974,-0.0310425683763944,-0.135378874395778,0.408799373478342,-0.37922799637117,0.168164497610085,0.248644583876443,0.528058503693413,0.177263467549456,-0.12237525109982,0.546678509524857,0.640400098893321,0.487025655229911,0.831333914042139,0.965102620803313,0.212235645973324,0.5517369476236,-0.792187151546735,-0.617608580937847,0.548333736291645,-0.467479565487739,0.883266393752041,-0.369119852456855,0.0779146169262155,0.36025417904673,0.663325939029419,-0.97636298636487,-0.283364878503891,-0.0936393011692795,0.378225475621923,0.257609708377456,0.987592581528058,-0.129787190083887,-0.0233021894094974,0.898607021444895,-0.0148579643302491,0.0573886037534104,-0.116670570431852,0.487704793047707,0.733766691804721,-0.678200767851026,0.531398864592909,-0.0508278590877687,0.261726080658757,0.893267026396804,0.796612956009201,-0.618188025959299,0.803053474724056,-0.895957665500691,-0.545466870053812,0.949655325209632,0.226507153954057,0.973053925710877,-0.309328193250757,0.864597368937799,-0.298858764999396,0.700002505191059,0.643570915183076,0.292426050956587,0.82031796100209,-0.490507708627703,0.819643732257685,-0.124734435400581,-0.465050133032741,-0.264916687764738,-0.595530519797275,-0.80255540499647,-0.0858239783609434,0.554236341671835,-0.491065993321145,-0.664269055927871,0.529984126194549,-0.134225908368388,0.251824500596795,-0.750574571159004,-0.942868442505044,-0.522105702227362,0.587091520563404,-0.281678481572515,-0.369286685960257,-0.862897041947623,-0.440788342977262,-0.30713013339364,-0.363955250744199,-0.16375291863101,-0.716876296356873,0.306067617879512,0.097257368990046,-0.241783944089819,0.60959692037515,0.118524664812745,-0.285099055000528,0.632215567635629,0.520790965521074,-0.905430075377757,0.556766055633582,0.267492409808595,0.11027096908772,0.603724392045282,0.257713017693387,-0.106608306767187,0.184787321297991,0.642326545957972,-0.86309917550595,0.177672354081546,0.370250325096022,-0.824004433029046,-0.587654123393628,0.201715958656462,0.599808877449624,0.362531894693795,0.299174869221675,-0.0980818887806905,-0.129564737707633,0.384788504150598,-0.68706338154733,0.939610390439206,-0.00335413999909973,-0.631869439794264,-0.829213267896372,0.342073949243589,0.00788628012815007,0.515669117604374,0.311792561369135,-0.304795684109387,0.300097961980038,-0.704733325367621,0.123675520145914,-0.301688325893157,0.988056626083066,0.300129107891698,-0.814558756329998,0.0995732851433471,-0.35443164654405,-0.490351237069482,-0.340217312892635,0.28086730455344,0.441578067573345,-0.219449571855156,-0.958215402878331,0.97256444424704,-0.529510220822845,-0.821092944140613,-0.83775550001858,0.313977032504858,0.881090898899056,0.453762067699746,0.957753408029998,0.152102644132497,0.144059494811706,0.402913400663789,-0.311307029768545,-0.432758301786862,-0.75246087980192,-0.855429030702021,0.574608582809687,0.545173938773449,-0.414140354617423,-0.0279298869337179,-0.673488712373045,-0.0667548618907272,-0.851657620025845,0.528651490808632,-0.981175956839668,-0.956790381655651,-0.554024757758274,0.521420344412059,-0.655019305143604,-0.60543937380989,-0.243803647504595,-0.666344215445678,0.647680667914621,0.288980667119612,-0.39854580519192,0.700749358630098,0.558923339859624,-0.18275520951144,0.556336757341683,-0.150879894686397,0.919551317298291,0.363555704223049,-0.906224815364089,-0.0389660389912143,-0.679797753904008,0.426138787658641,0.246786204219682,-0.144859404666927,0.780926627892178,0.156895705596984,-0.85060093871205,-0.858048345119353,-0.327203398413228,0.995766268061699,-0.267716138789148,0.60609642153623,0.470325633087506,0.579785412550798,-0.960334307028161,-0.561161884830583,-0.340701969738633,0.72928295848621,-0.197548968987819,-0.977678602276444,0.0180182664927313,0.231653187151745,0.745731798389165,-0.348580160460855,0.587767813099014,-0.499889796757662,0.988482961908467,-0.649778948670907,-0.0766885093078891,-0.340916083914172,-0.665325334120054,0.68231292746545,0.494347844300193,0.194273564138147,-0.294870533447135,0.612572180167092,0.62897055533515,0.285909630740873,-0.189169494589817,-0.470558172089022,-0.277230285366682,-0.558439006813782,0.651226779910206,0.125080179347706,-0.956770208304458,0.145164035598236,0.829775983261736,-0.255669073731223,-0.319171593564751,-0.662054584284865,-0.417951547811149,-0.5989857541093,0.109995210968399,0.936507670681594,0.546109241660029,-0.000817897924526356,0.212538052690582,-0.705974696194842,0.568214832852725,0.650749969646951,-0.489450523563429,0.508763244151631,-0.799690897417425,0.829845953730903,0.7670328126056,0.142808132773845,0.90786669575262,-0.158391745620632,-0.806821203262924,-0.20481210119683,0.444673586164793,-0.0812481234598477,0.305848585057081,0.749617607925879,0.951371982350454,-0.309242310435174,-0.110025621421804,-0.678582876425592,0.142166099251857,0.52776208348968,-0.55391028149148,0.451163383271241,0.458194387686935,0.460347889874196,0.228385009186046,0.28584592260962,0.496621790080763,0.972769835456924,0.0863512744799066,0.965462413272983,0.672736272402402,0.438243679667087,-0.876320753404973,0.870780924387008,0.487387724665804,-0.554931116632984,-0.64521431937906,-0.748541600620556,0.385104198944191,-0.31042882192603,0.127709259799113,0.818730279366894,0.360244163066598,0.413652837114775,0.294857651965059,-0.16936622494316,-0.137982370166954,-0.895942194779711,-0.925601579803638,-0.71587957692395,0.032715771625246,-0.562458788654648,-0.988045511930791,0.526624292654378,0.711921644714517,-0.17858299617254,-0.110421570082246,-0.635450112319511,-0.182427605680773,-0.65389600471382,0.296337381205274,0.0334095218421007,-0.979604038264168,-0.874133254044674,0.985226547296952,-0.639627657979023,0.846816523465098,0.159632681136737,-0.60964104414455,-0.525374631431418,-0.626340268883464,-0.599598180664695,0.326004276244952,0.0528972886423393,0.780217583613552,0.247757335696921,0.76284285538721,0.0627853116205133,-0.340639671265256,0.675280215560445,0.8561068846367,0.563622137541345,-0.99683493041511,-0.838188032422438,0.770351656069578,-0.820962692471345,-0.676277191895659,0.516762482483653,-0.961673964327692,-0.334886956625198,0.135577796830502,0.416821594949903,0.695792179577597,0.659379869401853,0.530829782912444,-0.789574103476565,-0.918719503069478,-0.327659231083258,0.0465792714751956,-0.915485361015828,-0.266566352289644,-0.707794812394609,0.904104902821193,-0.89389340746893,-0.762942583587854,-0.590043479615551,0.0269399966624342,0.210845126507501,0.213578375306342,-0.845753717277107,0.643289784924517,0.247297533465689,0.611381901903067,0.29614288170477,0.00988810703966814,-0.231354416112091,-0.400838255653738,0.176738092876274,-0.105802898010434,-0.920590580734881,-0.768212170601692,-0.272514855161633,0.22193394384059,0.206640710826098,0.532315511120713,0.866937074938346,0.534586083350995,-0.769578005627765,-0.327722314236909,0.267269431465715,-0.923116334867571,0.0479166202978194,0.407608462299493,-0.441741092886957,0.939410841235063,-0.710841574932652,-0.671666049149246,0.654741555164233,-0.951867291173182,0.310089806679293,0.693625315933634,0.476708190266944,-0.876330396165105,-0.901271321218912,0.538167504411077,0.947400665392525,0.539557859490382,0.348356773017101,-0.154039503811541,0.751501225695568,0.586438968119446,-0.0937480138447886,0.0542901503319595,0.963546212601555,0.611120625148737,-0.390576714431213,-0.977318428204597,-0.269734610388021,-0.945092975093886,-0.564895861702782,0.463154896025969,0.331182793312862,-0.220551353151175,0.556618638082373,0.0559081350582789,0.0708822621922276,-0.972679876743389,0.446463938940036,0.599515785731452,-0.0575207055054066,-0.068637187378519,-0.0640923871279322,-0.985787016198471,0.586146317269189,-0.66375355423826,-0.705222034996565,-0.631351786013783,0.881725464882106,0.396479325827706,-0.96013884403418,0.973954403087824,0.361699628313164,0.599712657230181,-0.236257290563227,0.988955730197585,0.465220011167383,0.794603945565898,0.40914266475324,0.0110851183150702,0.515769814565393,-0.472376664252149,-0.388988449946407,-0.414920029140211,0.293766668718246,-0.243879265336898,0.327000805049378,0.767414996609939,0.87340914573684,-0.195754519358727,0.7783628600187,-0.696478767655405,-0.830229582080087,-0.15732608590278,-0.378001925148828,-0.0631711031106743,-0.840056203056691,0.666456908145165,0.441800714313803,-0.736955229896042,-0.320478732472614,0.457758359502776,-0.685898938212086,-0.476319794780826,0.183526031225462,0.237662374484838,-0.831845179783638,0.672216374366368,0.70023437369843,0.257390967546151,0.00431059095942987,-0.640398650242835,-0.601602284090072,0.527280915706802,-0.730305513359042,0.90728763089384,-0.939721857170462,0.933966380598293,-0.459677086133869,0.324566722611205,0.779681693704661,-0.469245584614549,-0.146608812803665,-0.104435930594061,-0.460620702837037,0.285534383298485,-0.95298338161181,-0.247566056306413,-0.459146364564272,0.334407161759464,0.484602200960273,-0.982550067959927,0.772875133343733,-0.28751387713258,-0.218262529792566,0.0506927112195594,-0.85302353532579,0.73910265392047,-0.403530486615215,0.431513454808139,0.529867764910907,-0.76749816648492,-0.753318865249843,-0.493006928846228,0.370124072060555,0.727248256452934,0.889703845075032,0.491788433672998,-0.719530785537027,-0.890140351005674,0.233789148599894,-0.349449260297581,0.0890693584269762,0.702490801852227,-0.342636890644024,0.30376818561109,-0.220826635320255,-0.595748240607792,-0.204337207603521,-0.405750973844633,-0.428731190558726,0.0365424317336542,0.0021372428555253,0.988872446836263,0.95312795057157,-0.460684532986753,-0.877383291368824,-0.474187057953761,0.0292477233837152,0.301318540539682,-0.312647035687172,-0.767875965565124,-0.217761609276383,0.905574636580037,0.420474021434892,-0.00026519934589575,-0.568584715571304,0.779121981668322,-0.995604654306817,0.269439068647916,0.586648518217941,-0.344312611969815,0.458281957283077,0.0186932794097177,-0.210353218288369,0.433205886975802,0.499319860832283,0.616652390367781,0.323964279768734,-0.713899474636649,0.855093813913185,-0.10448197189153,0.37171585849357,-0.166422690179234,-0.421434572098176,0.843225515979788,-0.214655621600109,0.0794460705046236,0.696994565183511,-0.137450147361623,0.586490014320688,0.554414370663385,-0.614119734250835,-0.682792999331615,-0.462643833982927,0.792102661704189,-0.0955359411684449,-0.865089261458854,0.511590212511346,-0.122821059138869,-0.26209240966743,0.596458968853454,0.660285106746309,-0.898513283882945,-0.241569319821429,0.0851728345979468,0.121662574998103,0.658227158144963,0.194581192288706,0.614121174587452,-0.372885575110284,0.491676648891614,-0.628682486061675,-0.291221488071867,-0.630345572641852,0.497727029247754,0.0369338709776832,-0.375027447182032,-0.939233352931761,-0.487387056559362,0.6553032634768,0.494292303430044,-0.956246773150412,-0.802979557612201,0.966953148697981,0.874158794246932,0.681794172438646,0.807056721248335,0.654419392243615,0.643594842909541,-0.625733834056477,-0.510108409917995,0.529614928694445,-0.584540001300377,0.947765140045143,-0.355634380684243,-0.148173101527805,-0.240185449227295,0.21095746473262,0.286401471246641,-0.846777083719539,0.000749215259739255,-0.645330914463757,-0.304240389111069,-0.0707004255679351,0.700389211915116,0.268492483960296,0.227797817511386,0.650157366032285,-0.232011676945203,-0.404410539641902,0.00410942196539654,-0.865590797902215,-0.674895505430184,-0.919225927826109,0.205319239613028,0.999052538793509,-0.853345819622267,-0.557630053552874,-0.267289280441259,0.725952971355418,-0.518338850548588,0.160616580630639,0.239288436449755,-0.754044833512842,-0.396190397045518,0.916910052502229,0.275730831923827,-0.929178502374486,0.679881412017615,-0.229228869547317,-0.105169271407477,-0.289018506516657,0.647194269718953,0.767534242896517,0.744571036660418,-0.759754009161982,0.548292401497565,0.647788595658662,-0.0737195834051432,-0.0400893667566569,-0.162142502518547,-0.82512255077243,-0.298381745502876,-0.800519768639443,-0.618095988525489,0.32646118602443,0.823724605401698,-0.851538365982863,0.661328590416117,-0.657580991434357,0.00950477147077167,0.0613842470182671,-0.299324561092003,0.299255643814795,0.530496415128127,-0.559264561480062,0.032349479729767,0.839709724952199,-0.877516056030791,-0.437187471708887,0.759839055856769,-0.730450399633525,-0.452069470145434,0.920358952523847,-0.704426998377093,-0.599766461648394,0.502929401841761,0.858300210689296,0.172959298473804,0.0542957763179945,0.0546723103395366,0.497598379838624,-0.462737620103466,-0.0371820137376428,0.227998579079328,-0.180350391473539,-0.421732771604177,0.150804513391808,0.363925055205998,0.205622759693721,-0.644786034692874,-0.976470108017867,-0.653140227087536,0.877129279046251,-0.740160558704082,0.122840444923737,0.0456216475737234,-0.0217579595837738,-0.421537020726857,0.645465535120555,0.42191858210556,0.363754939535238,-0.680794387034403,0.00428931385394948,-0.171674734975546,0.396894854262058,0.67673640709371,0.165277448004196,-0.622821287597233,0.0947571735113533,0.444121706781246,-0.309158268475013,0.356127963567895,0.363102192786194,-0.111386942677334,-0.815474974514145,-0.942269237084684,-0.336989697738639,0.305355629704931,0.0024591924456403,0.335543930796244,0.673635988441368,-0.415674044309955,0.323600003077329,-0.956190641806222,0.873042275225166,0.73726421528338,-0.748636212486525,0.37351990121268,0.490312646506288,0.583026378548857,0.735948214102139,-0.652815371114504,0.935473538967729,-0.497910264710417,0.926492676846204,-0.58089232005722,-0.92421964486878,0.345082729350643,0.934550982984627,-0.170637789075439,0.848348938591551,0.689863967741572,-0.837315882129641,-0.0924233750575618,0.931662551658981,0.599647402219813,-0.533666804551314,0.508874239714043,0.140787611519937,0.661475439825197,-0.677063670938074,-0.00452682978491681,-0.371826944359137,-0.57589226775197,0.89646654046316,0.9061116701854,0.939954545700204,-0.425321178611264,-0.121147415871277,0.063962744510653,0.378945834955972,0.0159782478642212,-0.81348924792272,-0.628613819457605,0.225087310757855,-0.949801167963444,0.823008851155587,0.806437575753202,-0.0929069836451599,-0.615651344937618,-0.562054004620092,-0.115831413407202,0.496701497186146,-0.0619806652356121,-0.249755560860699,-0.850003893140079,-0.893305266979365,0.79158785666727,0.768343961287586,-0.363473041897812,0.127042143036072,0.582762970654013,-0.471257157751319,-0.117102877443415,0.574642700696072,-0.774185668504646,0.612763190121694,0.486514571459007,0.84889885880173,-0.740161110362298,-0.708317212886332,0.193490383438335,-0.603772870012977,-0.899711463272935,-0.21957638282868,-0.874840148228937,0.98085133865824,-0.997627541224228,-0.314499854558502,-0.48378104923662,-0.139736464517049,-0.610275111841778,-0.588763074693897,0.639229271847422,-0.409048155614196,-0.633467687955944,0.656153243031041,-0.580670038992427,-0.00657821933123537,0.584398993189844,0.690334681755454,-0.672366353581415,0.573296268986709,-0.256625631740834,0.242706575430842,0.513569285578404,-0.409447025956552,-0.310398834483656,0.718698456854008,-0.214321268335038,-0.668507011468633,0.676682177030617,-0.211221173355598,-0.513972162735281,0.769528840731703,0.47032189955172,0.941835702024083,0.0119363667258632,-0.215234932129962,-0.877781025150613,-0.314093409214903,0.242939959721208,-0.680279563295928,-0.904200862926064,0.629396658990715,0.717313803871836,0.802959612336879,-0.87411260884242,-0.121806854509309,-0.381348170334873,-0.265336560655044,0.703920161917776,-0.473864333909276,-0.669654474080239,-0.389778773158851,-0.469528837762377,0.00959505992582477,0.640164897450308,0.108089845401703,-0.272998846801957,0.574662327332518,-0.0308179619060143,-0.426656659637571,-0.699298966094931,-0.0127355038082715,0.304254788497797,0.630615513781966,-0.371123378606761,0.208136315650762,0.805772226330442,0.906886448749516,0.399957095249484,0.207567420972744,-0.688156105449262,0.695695674621413,0.282418440300957,0.643229972132579,0.356437179098536,-0.362929422149726,0.633356440908506,0.416345844766608,0.417635431337466,0.580128564857042,-0.59593365690035,-0.262171823899476,0.999207876267378,-0.765301130064861,-0.0890549345209128,-0.609069319582945,0.735204388687564,-0.378507033747289,-0.0715220782150854,-0.0824466498578209,-0.293152905272431,0.812750842811939,-0.427672041228427,0.568343397197623,0.441715268876513,-0.897673700184587,0.217216630715447,-0.041933321039797,0.578188625855726,-0.170178362432491,0.610037920274903,-0.287710630442475,-0.215903188870732,0.361089759837071,-0.119869563860117,-0.571947999828138,0.804527858624247,-0.191313107280004,-0.317409736828641,-0.913730578792213,-0.394782564995622,0.0111666277192273,0.206618000043035,-0.592256994692942,-0.900583855677534,-0.710899249928472,0.00405071474350915,0.253709283544915,-0.471646232570784,-0.205348716230258,0.0983116929793644,0.326760908216362,-0.251340000242344,-0.963239292704307,-0.0411192828798144,-0.0286347546533909,0.464201032656389,0.706316638037629,-0.04257590383364,0.984004005574744,-0.537972778038597,0.548298737343452,0.762156733123627,-0.559676434244321,-0.754882070714443,-0.739672940835424,0.879130565676944,-0.782687598521714,-0.971350681400266,0.442704016821276,0.800554004958469,-0.338122926781324,0.986712448720539,0.961762730340856,-0.446854310725769,0.514664218351172,0.554188684425139,0.132634473650853,0.338675586854914,0.836867588038477,0.83427227927737,0.764936309995564,0.976576242493318,0.297085566507181,0.252285188050452,0.394993289284514,-0.370287310685595,-0.00663684955630695,0.256748439506322,0.668525243704359,0.714080176048335,0.244621331378636,-0.00303869827739578,0.487367014879246,0.543383080163887,0.0735952324433267,0.313495449416492,0.0724935527023114,0.72485533178764,-0.884403297766596,-0.616161190933313,0.0434004668457943,-0.0807449918401302,0.383572243501463,-0.68484268880969,0.546330786150202,-0.794459188364503,0.97084683850002,0.924902070906054,0.115666414538233,-0.356660082718114,-0.405429187241038,0.527610519510764,0.00481918064793074,-0.270118471520895,-0.924178293465047,0.510661446524852,-0.463589264597503,0.972815936429492,0.769295156497708,-0.335469685863777,0.746164647386917,-0.664160640640239,-0.494524805850822,-0.977851835895432,0.108279691082976,-0.987082872167938,0.928136828167559,-0.124107367484763,-0.739645467826113,-0.221943151594525,0.0442967650209107,0.206015699826542,0.359400182983792,-0.0426658231158263,0.524576406943227,-0.808403095762363,0.0693646038265174,0.562967865063796,-0.777219680601422,0.561108781636107,0.575212743250324,-0.0782834262653964,-0.312440259471308,0.256685081547891,-0.729191952897368,0.16337459153103,-0.962730356478369,0.0211693587955941,-0.911045146877463,-0.0897735389785879,0.0999218531138719,0.801276338051642,0.329117476939966,0.600838892523439,0.697512331110971,-0.187259911169497,0.921501823921593,0.175434254828807,-0.0910520160033421,-0.625980529236019,-0.37924791696042,0.287207189814183,-0.0501470799571648,0.319692738969241,-0.103785944708615,-0.451154885420365,-0.694333184074195,0.491630116784142,0.949884338465322,0.385832902791249,0.95803671266723,-0.933149965401021,-0.705060099275972,-0.575399834185748,0.188145069611646,0.750825327669123,0.672962958065169,-0.0569020839570962,0.262814205529991,-0.052673481287786,0.624303492359213,-0.229589182066159,0.171710289449457,-0.560768767245649,-0.796344351773823,0.149178413470678,0.101151836916934,-0.011949949645254,-0.526756796181062,-0.735794934156698,-0.0367344730323589,0.00912678854608884,-0.153038718556749,0.486589603880176,0.183573872203237,-0.687012280851619,0.440076564834491,-0.733694978761056,0.216301748282254,0.307975536766331,-0.451575800950739,-0.897907301506147,-0.0546569786197619,-0.349021891553052,0.62938414485097,0.0562801988799921,0.732551307759657,-0.640997394582714,0.824026063501267,0.584125380407177,0.0655964219046279,-0.529374412400436,0.478253866246308,0.662520883431695,0.326597537040602,0.436842198350736,0.595199605687614,-0.218405575799845,-0.225185296677573,-0.174502887929711,0.74932542054598,-0.364628997775138,0.434757629956171,0.936428364928162,0.639358072403496,-0.0707426534416924,0.801525127522926,-0.523401376547358,-0.678047148597764,0.104712647960532,0.0126661006381243,0.924495979493975,-0.111745041919781,0.920456294567584,-0.212972731866698,0.167935919484479,-0.466219610271739,0.625253585548672,-0.922800641849089,-0.762424482531113,0.927742034809444,-0.791839438646436,-0.52877638262086,-0.900878558584087,0.900037599741399,-0.523523996224993,0.391510128209365,0.303516590382966,0.818317547197376,0.994165797219216,0.797207201706803,0.892795517605705,0.606720405621049,0.136852240806799,-0.535722371329385,0.0361660061966478,0.270325130655909,0.430204091948188,0.230388481925075,-0.435390203420374,0.556724970131376,-0.904177792918591,-0.625504242266023,0.77139975274338,-0.0586729448682124,-0.743200149587014,0.762880712283539,-0.00913200968564021,0.210532348363328,0.775276380103609,-0.392442243609459,0.912905208719205,0.659154505510811,0.0705550779152035,0.697277480625742,0.668531293274583,0.876075765268676,0.462560992299914,0.565206027362446};
    cv::Mat bMat = cv::Mat(8, 150, CV_64F, datab);
    double databeta[151*6]={1.2554854546083,1.14947267558955,2.15487023814214,1.83814708518973,1.80289243979906,-0.327207753758491,-1.62581128860044,-0.650729656231496,-2.87169699294847,-2.46703370784795,-1.37572005273203,-1.08262996135853,0.45845754598667,0.291016645794131,-2.08209294109905,-1.6411291948773,-2.7972379587138,-0.792472253612706,1.52033571411975,0.905170206484739,-5.59422229368993,-1.38139166817608,-0.77436795678694,-6.74102509951844,1.26412218202962,0.246114926857144,-3.43228644024093,-3.28789245551405,-2.42865995665296,-2.80693236214968,1.31677092079673,2.07936118468882,0.403693022534597,0.486117395121868,1.50021926253655,0.991055377109008,3.07586081655923,1.79875120949021,-0.163171551932263,-2.1903719785502,1.64102741961366,-2.55766424059632,-0.556298231965215,1.99685016247021,2.70452002505872,1.97471506258936,-2.87785035175092,-0.612504926962298,-1.50052828116464,-4.72204682343047,-2.83308885932675,-1.73972498685215,-1.46156072434371,-1.4162527965094,-2.49728864785542,-0.743136490606618,-0.930713292776962,0.433157462514054,-1.29910611417411,-0.590337864520274,0.317581152041459,-0.928435817859163,1.07283867853862,0.179245648041581,-0.402060983628774,-1.23573204891693,6.6082028487027,6.39973754387758,6.18199728147158,3.64330546232493,1.435190224298,3.54136613779067,1.17116612054662,0.448135350897485,-1.71876487559054,0.82330411524265,1.22801956285859,-0.180711260546796,1.42924067032217,-0.861294443426661,-1.38240588547803,-2.72161151907279,0.732083679872574,0.1656799886547,-2.88298630351427,1.31622331763923,-2.49061021868949,2.41982099586244,-5.11806543266168,2.59739893959,1.20309649993439,1.20026330699498,-5.20534711248795,-1.29419549544898,0.938541165179721,3.44144600799894,-2.1086854265146,0.0128026808331066,0.298945433343994,1.53855475398644,-2.42910001656479,0.36168018454778,-1.44528695399616,-3.96122009516839,-3.21688875650462,-4.18075225894333,2.04889476179389,-2.00105122205813,-3.34041132010126,-3.60558161610157,-3.15452265140304,2.03113651839326,-2.97903898165961,-0.0862187422922267,-4.12492006270062,-3.45979593319395,1.9809637702771,-4.98576659907807,-4.52884772178021,-1.89846279126418,0.0579212023856121,-0.546164269889805,-2.99869163851445,-4.29491373948073,2.93277757491111,-0.136452295266423,0.36195569098254,0.0231777904970022,-2.55735777018558,0.848295535167992,1.67683692238451,3.29059076470949,0.0952635483301756,2.13258553142649,2.16901084424453,0.234459111375553,3.13770709023064,0.679681117780382,1.15986731884975,0.0809460288905961,2.10110450150248,-1.18670361887129,4.87761066949926,6.15734586096201,-4.01708142786146,-4.93354366601503,-1.5313870071934,-2.76868007200844,-0.283493305308172,-2.38299153613188,0.815035355386399,-1.3710758305302,-0.971625635303082,-1.17006211538547,3.20381448154443,-3.25173817501232,-0.657929832989867,-3.60218063352396,0.513725392083845,2.80789141045264,3.78602843275816,0.052016698440667,-1.23644910200339,-0.243429606888366,-1.39188428290097,-0.430293670792972,-1.14538560013662,0.30819308051335,-1.23874966743749,0.238748201745641,0.958093770475098,0.191471162687389,0.467517896147391,-1.11590730542153,-2.47016862412587,0.404401269103035,-1.57669178676533,0.542880825307677,-6.17151428617004,-3.54629311842523,0.699611058188176,0.347427907684826,1.02919058958093,-0.0689233860313293,-3.73159527436058,-1.72587321476246,0.752903047419326,-1.1633771471908,0.934853317858173,1.08392134817369,4.81514564464259,1.92322117730666,-1.40345571096242,0.0824069889493665,-0.343565035068577,0.00575205544446347,-1.96253631564179,-1.88003747530155,1.96086316893295,-0.333224180877314,1.46433432992827,-0.629366529080842,0.250864926887508,-1.81472408292672,-8.61863284302858,-11.7710739843958,-1.76185603437394,0.828022548646729,-6.4190131175301,-0.313754559311294,-0.290399425092491,0.03914547239117,3.5512473180499,0.813293746543196,1.87677473382215,0.0184401645170832,-1.37001188952961,-0.30306535933607,-2.33073987606641,0.475012518521862,-3.24543884796301,0.687320742997395,-2.0291586119,0.466060538910055,-1.96139646094616,-0.664571380472388,1.89185544866967,3.29095886499054,0.23322004624013,0.225078509891296,-0.60808301580661,-0.875075136063527,0.976941755880326,0.733790426747356,1.58714200384788,0.239144007757227,-0.583531319588416,-2.85375879346215,-0.202957099549736,-4.91015843789518,1.041763681185,-0.626900344399906,-0.57294590555342,-0.990598430804486,-2.31406569776885,-3.8849354694961,0.043049765643155,-1.50764250549062,0.896704503515718,-2.75704063074391,-1.77186787563217,-0.467000721001626,-0.725218301481256,-0.285453759122886,-0.969463118311214,-0.74645034134842,-0.93590501281399,-0.324207349338635,0.705826527376074,1.18850915785297,0.397418955822021,0.599008467760636,-0.0364227981833619,0.0299807844797575,1.564174334989,0.781009561306106,-0.498337175159401,0.357984161875261,1.20072198953675,0.928975669365482,-0.130916951310424,-0.853109903617654,-2.49486374104188,-2.84750817702811,-2.28121930739053,-0.171925903251846,-2.60265003597944,0.851265402725288,1.1911721195041,1.08252871478904,0.0427422799825524,-1.00058746096828,-2.18667555248496,0.56781828441521,-0.0634378840904066,2.68643972267865,0.943032567887672,3.12616582521354,-0.91504399583073,-1.10333810591771,0.0420536841681047,1.09007493360851,-1.98702726161798,-1.51634674950859,-1.68373120424322,-1.57797380553985,-2.1273180782678,-1.17973367785782,1.04832272653817,0.0724954193397937,-0.495416178124379,-0.753433151610365,3.03568152951518,2.96016204423349,-0.789615530175705,2.421405441111,1.82545492117804,0.299391183653936,-2.69415797100214,-3.54507338720074,-1.87929271983728,-0.741895033341764,1.20215306392453,2.33988471764706,4.13411569443979,1.46171515082872,2.25629014339032,0.0444976348559072,-0.149887553061099,-0.234223622767491,0.961649073488495,1.26731160169552,0.318454756628867,-0.415609567762622,-3.75327472425983,-0.141369336530631,-2.09869934728208,3.99942864708416,-0.282147801152844,-1.83736352695049,-0.179449231608024,-1.07489570275526,1.0916440688449,0.449674149673042,-0.602821969688825,-0.689127624639439,0.86160066997357,-2.35774740803277,-0.632221164971423,-1.31780815991802,4.12309585435367,1.67507041187283,0.586387029815125,0.566591179818559,2.09368165427692,-0.357755824718321,-2.01931951407595,-2.15813879714688,0.872139594096874,0.428549191545022,-1.58596053791882,-0.779365497779914,0.166387588022997,1.11829425577448,-2.33791559262455,-0.870591716238574,-0.702062915052423,1.34382185205627,-0.675869424319923,-1.17862687864349,0.0158881450441116,-1.26746280117112,-0.69016475660963,0.665486676645162,0.0251161693784346,-3.39602629486859,-1.40689487777518,0.264127130007617,-1.50952195511637,0.673200595363282,1.18049887932622,1.28463574051962,1.52199385213638,1.98320580135538,1.92464094558233,0.534427495961817,-3.35934151885304,-2.50860168643861,-1.17576873676649,2.12364906924494,0.796867673150714,-1.10308238196784,-3.74831316432138,0.0423511015909725,-1.06190592207807,-1.19980720965737,0.271223110602751,1.23053589979596,1.38156377549052,-0.714931503485942,0.251833621168375,-1.15757518631108,0.765695504606276,2.17298800820358,-1.07224079849395,-1.11704251322851,3.34870058379052,0.443705514147934,-1.4671646639096,1.96417515848838,5.48444446398008,-1.4954420944489,1.37029480991562,1.92316265663814,2.10045999140223,1.587074739497,-3.51448611191832,-4.36788418759014,-2.79786988612873,-0.974563950695079,-0.735641018983342,1.51425425758566,-2.54989509368159,-0.33043312106428,-1.98587091163317,6.20654764600228,-0.0650172398601222,-1.78273865843759,-3.53043152363048,-0.645917646660395,-0.801358296787321,-0.490765068263147,2.92505630602806,2.9073473252304,-0.555403675880521,-0.493585624850082,3.01039904602704,-1.76894960022187,6.92263896053289,3.75212210066458,0.837003846661611,-2.5543653604094,-1.33138418586034,-0.294079239106904,-2.03031300910176,0.304604818712438,-2.97331578873656,-0.520785265437809,-0.165197865803368,1.14494376773959,0.0290560389338608,2.46197294672811,-0.0986702958136394,3.15143133769082,1.06704732443972,-0.421472016241638,1.19116895632833,-1.75911485217519,1.52088172056447,2.11749529568552,2.07832001453845,-0.88051459816001,6.67099038026716,1.21400107098304,-2.09915194411543,-1.04827051750995,3.50025183546502,1.34787240693315,0.737542783484911,-1.09293580081071,-0.0816338580484095,-2.1240841353242,-1.82025827517115,-1.14030978664588,1.02906360971076,0.268731228820081,-0.960060348330505,0.781724662102422,2.82419892407928,2.55160292795257,0.99809521364241,4.47590569769886,-0.669398349123484,-0.435176477734262,0.279229132309894,0.42228054133516,-2.79154406146013,0.22083512408924,-1.86057744984662,-0.340325942297584,0.528685015453569,-0.321933956512141,-0.272927741181828,-1.26921979094655,1.30453347635797,-1.0928216906765,1.02027856630392,-0.389241695868035,0.817866403275905,-0.20796042252449,1.98225100488807,0.156310722580559,-0.78811689707308,-0.594466565461487,1.4226854606164,-0.782613308639219,-1.44634513646138,-0.255423770143935,1.14202840189331,0.537103578911022,1.99214270619348,1.92602288017149,-4.17020155774328,-2.22518263852467,0.460952224038543,0.0967241138279007,0.189041684456359,0.55874278019541,1.58478241175152,0.412191249521911,0.471911436770843,4.30981698304374,2.22181037293472,1.46509471319566,5.72574409871533,1.56204697701308,0.192134593086418,2.89038754113971,-1.7781831015499,-3.29741027525996,-3.92336558088791,1.77457555975264,-0.338787950198244,-4.02353591091536,-5.57410057159606,-5.33585400825648,-0.690300854141359,-3.83275191416672,0.641245241667205,2.44623435903694,-1.39124806209469,-1.20494630498467,1.03418366161781,0.48960281857624,-1.03421985460294,-0.990596801433389,-0.892555268749142,0.754396129265747,0.956813347724521,0.732423941058617,0.782072736914604,1.23132782043735,-3.22247082096136,0.793894884357878,-1.70146483062221,-0.917818326306757,-2.65349550509828,0.227032503242234,-1.27125635488809,-1.84908325765972,-2.52677247435061,1.09029674376146,-1.09888474170628,-0.0632939662261431,0.464975361371518,-0.249295954712805,-1.988497500965,0.213918356421722,0.218176256287701,-1.92575257819067,5.24984370783842,0.589683454868091,0.375496935053791,-1.18832280947844,-0.337338631614731,-0.268892258778129,0.428692128300943,0.968865799193131,0.169211120052884,-0.832575705095103,0.532200382596393,0.366283838773452,-0.401147145104717,-0.549523309639495,1.06681546009956,0.858069819845065,-0.640556119177268,1.79656987676642,-2.78785609900263,0.297224973804698,-2.06984132916598,0.684920580099334,1.55221831657632,-1.00977715689357,-0.936544357699184,-0.669329677619469,2.85006246639672,2.71039313140854,5.55643220204523,5.1421047633703,5.11135566596389,7.17620751313649,0.220787974951282,2.1673405661224,-2.99038164505975,-4.07957616964447,-0.941507443678527,-2.04443322513277,0.404872385881491,-2.03627650768175,0.0899206966148386,2.29106547900314,-0.445240009437162,-0.12446689525541,-0.639642070866764,-0.297110630952919,0.920711415173351,2.4018196833502,0.780211187559408,1.15786002573936,4.3829169294709,0.938025765070909,-0.849109238115699,0.0595949405596317,1.52690149279522,2.0812895654196,1.11580042026874,0.473661840207214,-0.644580088700474,0.288819317763923,-1.11916899048375,-1.73120491531406,-3.16602978457866,-0.938557149812505,1.58996974289799,2.25881794547076,-1.99189939056633,-4.28148031405349,-4.58282975942325,-4.11965409045888,0.984260045759781,-2.05051650279837,0.700376182576961,0.137784361955479,0.369514544394509,-2.02500171225379,-1.10827565355209,-4.02960091492343,-2.61459959052234,-5.50005936267622,-2.58778530532172,-4.76635411049148,2.3131256836601,-0.0712083083428666,3.4852084941502,0.643322232170802,3.03935398830889,0.213718817772759,-1.38023458464569,-1.08639876873485,0.467288022506636,1.01080402747797,-1.40891706186819,-1.8660809121836,0.474544392111635,0.277425187579994,0.136194009579864,-0.626076124698813,0.12724627581002,0.629484677729289,-0.962867688576013,-0.613998826622794,-0.921351352568832,0.400219632546267,-1.47848456525788,-0.368479248924665,-0.210060656993712,-0.374395149550272,-0.79035862300614,1.08813356108658,0.128701412466627,0.428808196720245,-0.0768282033199333,0.828740020456407,1.30095138044821,3.85742809957139,-1.3820232622647,-0.395170946206161,-2.13709872986567,-2.1929854809002,-0.96992047126425,-1.90994577072822,-1.50463609343207,-1.66363450319358,0.907487353094206,-0.324433939661457,4.13547839072649,0.438582888006772,-1.47451697676801,-3.12585193726779,-0.48693744169232,-1.45729111521247,0.304226903529696,-0.686859038245856,-0.173290669902895,-0.624960041188121,2.70073025905032,1.88985182295684,1.51506105361657,-6.28473416026556,-3.10362944359161,-0.727781150228028,0.299988043247719,2.70224375278622,4.93207994040864,7.72903927389357,0.373847992748727,3.27449097988739,-0.379515605616202,-0.113889444577806,2.23981596654557,2.67208597683653,1.58567910944203,-0.403240094512315,-0.170640921615592,-2.2549996494385,-2.33132662249354,-1.97316728172661,1.69469162569445,0.112431493755767,-1.98091799246368,0.43202243393471,1.1003112736735,1.85475277800057,-0.704886272761104,-3.23150560533473,-3.39275844928094,0.621952831905527,-9.70609712873844,2.0478594778561,-2.88905708828744,0.0738547308502828,-6.22470166791258,-4.14717070498816,-4.84063220601987,-0.632459681908454,-1.18950313965217,-1.84532279864683,3.58915922406075,0.804776437671007,-3.76489116273433,-2.99139414613932,2.90161560109819,1.5144045386916,-0.543024830205828,-1.9053847102562,2.72840652353986,1.06844193669898,-1.45590888553774,-0.334271532103241,-0.208228244290892,-1.3372764513989,1.10847000752989,-2.08960874727215,1.62553714717994,-2.91230329452635,1.19536691353282,0.660762732471639,-2.98138959543931,-0.185671595473124,0.220532009774579,-0.958694008155888,0.49301695172984,1.35476615393028,-1.17406223794009,-0.797545426577345,1.22091368568616,0.768548003308185,1.45740961542393,-0.326943485160758,3.53180268350595,1.52465415842676,1.39075019671593,-1.4176808475803,-0.999495346038346,-1.40065090483546,2.61514496717244,-0.473842098703528,1.13814692551963,-0.28289434415772,-0.620335816877127,0.0632959273089397,0.0259414056234594,-0.403284576348779,4.60277494605557,2.9007549744505,0.577943276576948,-0.947231828037022,-0.773244193727576,-0.728647213003338,-1.53755580214492,0.0694712961157505,-2.70371120446338,-0.731588041434347,-2.55682201268975,-2.63685144616103,-1.1786694179022,-2.60569480029752,0.605849609591543,0.930697586507127,-0.333384456256594,-1.33250285780628,-1.33287954622279,-2.04544519062751,-1.04459735340386,0.427459472442877,-0.996439181512778,-2.45368825498909,-3.97662618970726,3.78633256031053,2.32286782361174,-1.99350322804269,5.54854060416827,-1.2531169939763,-0.0931900385128152,-2.26480159994872,-2.24873069458757,0.14126700423505,-2.97660853056796,-3.44449580301541,3.55526809624425,2.36752845323614,-2.46643834306278,-3.24950343716473,1.22527319993906,-2.53476919736108,-4.9930089812534,-3.69738065759657,-1.12475523944045,-0.100188158912032,-1.1421492581448,-1.33444805639247,-0.490020332375368,1.18768923240646,-0.389310998019045,-1.82605593540979,0.756667971640136,-1.10806623255122,0.0627365613595195,-0.46274099013213,0.0018801420818044,0.198097519404301,0.11690113583498,2.1443600849568,-2.26138804840759,0.434723837707946,1.40143872157782,2.23313683376986,-1.40967294705985,-1.5649031056403,-5.66396174961584,-4.74678291181484,1.47811050691797,-0.163510306576707,2.71264305130362,-0.253754891086312,-0.542795808765515,-2.63382773400665,0.80903518807812,-0.167626873173584,0.229886244095365,0.889712247814422,-0.867765789574247,-1.07343355284461,0.136109751605924,1.30594270366012,2.38051819899335,1.99669654807835,-1.27506088922658,1.29315020213357,-0.656572851239288,1.11305486087164,-0.0385560392060368,0.55492438953912,0.63238597140851,1.95307235289753,-4.09549888796393,-0.341833023792713,-4.83657161608959,-2.04380284098372,-1.84586929462629,3.80397035448141,-0.454857657019202,1.09318701422868,-0.436373638979082,2.17617372024024,0.0177920645688862,-1.2882719015852,-1.96962206370062,-1.50624111927694,1.39741722126915,2.05402067731967,-0.604142220800017,-0.709080455503548,0.904168822022018,-1.07248062165774,1.72759884177023,-0.436108820167132,-0.236955073642346,-0.022799386824183,15.8556503012238,12.7408826215417,7.38857548537993,6.19713462133634,32.3937392904348,31.3550589742302};
    cv::Mat betaMat = cv::Mat(151, 6, CV_64F, databeta);
    cv::Mat inputMat=cv::Mat::ones(1, 8, CV_64F);

    //Scale down
    for (int i=0; i<8; i++)
        inputMat.at<double>(i)=(input[i]-mean_in[i])/std_in[i];

    cv::Mat H = inputMat*bMat;
    for (int i=0; i<150; i++)
        H.at<double>(i)=1/(1+std::exp(-H.at<double>(i)));

    cv::Mat outMat = cv::Mat::ones(1, 151, CV_64F);
    for (int i=0; i<150; i++)
        outMat.at<double>(i)=H.at<double>(i);
    outMat=outMat*betaMat;

    //Rescale up
    for (int i=0; i<7; i++)
        outMat.at<double>(i) = outMat.at<double>(i)*std_out[i]+mean_out[i];

    for (int i=0; i<6; i++)
        output[i]=outMat.at<double>(i);
}
