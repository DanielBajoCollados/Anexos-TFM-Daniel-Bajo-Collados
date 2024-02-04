#include <conio.h>

#include <dos.h>

#include <stdlib.h>
#include <stdio.h>

#include <time.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

#include <iostream>
#include <fstream>
#include <sstream>

#include <Windows.h>

using namespace std;


#define NUM_MOTORS                      6                   // Number of motors

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                  // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_GOAL_CURRENT           102
#define ADDR_PRO_OP_MODE                11
#define ADDR_PRO_HOMMING_OFFSET         20
#define ADDR_PRO_MOVING_STATS           123

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        115200              // 115200 baudrate
#define DEVICENAME                      "COM10"             // PORT being used

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define OP_CURRENT                      5
#define OP_EXTENDED                     4

#define AUTOHOME_WATCHDOG               10

#define LESSER_MOTOR                    2                   // Motor that needs to be lower for the camera to see the end of the arm (2, 4, 6)
#define HIGHER_MOTOR_1                  6
#define HIGHER_MOTOR_2                  4

//Global Variables
bool SetNewPosition = 0;
bool StopProgram = 0;

//Function inherited from the read_write test on dynamixel sdk list of examples
int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}

//Function inherited from the read_write test on dynamixel sdk list of examples
int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}

//write a signal on the communication file
bool sendMsg(string name, char outbuff) {

    ofstream CommFile;

    CommFile.open(name, ios::out | ios::trunc);

    if (!CommFile) {

        printf("ERROR: Could not open CommunicationFile.txt\n");
        return 0;
    }

    CommFile.seekp(0);

    CommFile.put(outbuff);

    CommFile.close();

    return 1;
}

//Read the signal on the caommunication file
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

    Sleep(100);//Sleep for 100 ms

    return 1;
}

int autoHome(dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler) {
    cout << "Begin autohome process" << endl;

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    int poses[NUM_MOTORS], aux;
    bool active[6];
    bool end = false;

    for (int i = 0; i < NUM_MOTORS; i++)
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);

    for (int i = 0; i < NUM_MOTORS; i++) //Set all motors to OP_CURRENT_BASED_POSITION
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_PRO_OP_MODE, OP_CURRENT, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        printf("Comm Error: %s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        printf("Motor Error: %s\n", packetHandler->getRxPacketError(dxl_error));
    }
    cout << "Motors set on current based position mode" << endl;
    for (int i = 0; i < NUM_MOTORS; i++) {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i + 1, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&aux);
        poses[i] = aux;
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, i + 1, ADDR_PRO_GOAL_CURRENT, 100);
        dxl_comm_result = packetHandler->write4ByteTxOnly(portHandler, i + 1, ADDR_PRO_GOAL_POSITION, -13000);
        _sleep(10);
    }
    _sleep(200);
    auto init = clock();
    cout << "Start homming" << endl;
    while (!end) {
        end = true;
        for (int i = 0; i < NUM_MOTORS; i++) {
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i + 1, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&aux);
            //cout << poses[i] << ", " << aux << endl;
            if (abs(poses[i] - aux) < 5)
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, i + 1, ADDR_PRO_GOAL_CURRENT, 0);
            else
                end = false;
            poses[i] = aux;
        }
        cout << "..." << endl;
        _sleep(200);
        if (clock() - init > AUTOHOME_WATCHDOG * CLOCKS_PER_SEC) {
            std::cout << "Error: Homming time out" << endl;
            return -1;
        }
    }

    cout << "Return to extended position mode" << endl;
    for (int i = 0; i < NUM_MOTORS; i++) {

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            printf("Comm Error: %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            printf("Motor Error: %s\n", packetHandler->getRxPacketError(dxl_error));
        }

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_PRO_OP_MODE, OP_EXTENDED, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            printf("Comm Error: %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            printf("Motor Error: %s\n", packetHandler->getRxPacketError(dxl_error));
        }

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i + 1, ADDR_PRO_HOMMING_OFFSET, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            printf("Comm %03d Error: %s\n", i + 1, packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            printf("Motor %03d Error: %s\n", i + 1, packetHandler->getRxPacketError(dxl_error));
        }

        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i + 1, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&aux);
        cout << "Motor [" << i << "] is in position: " << aux;

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i + 1, ADDR_PRO_HOMMING_OFFSET, -aux, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            printf("Comm %03d Error: %s\n", i + 1, packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            printf("Motor %03d Error: %s\n", i + 1, packetHandler->getRxPacketError(dxl_error));
        }

        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i + 1, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&aux);
        cout << " , and now is in position: " << aux << endl;
        _sleep(10);
    }

    cout << "Homming operation succeed" << endl;
    return 0;
}

int main()
{

    //Set the seed for random numbers
    srand((unsigned int)time(NULL));

    //File to communicate with the motors control process
    string CommFile = "../../../../CommunicationFile.txt";

    ofstream DataFile;//file to store all obtained data for the neural network

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int index1 = 0;
    int index2 = 1;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    uint8_t dxl_error = 0;                          // Dynamixel error

    int curpos;                                //Current position of motors

    uint8_t MotorsSet;

    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return -1;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to set the baudrate!\n");
    }
    else
    {
        printf("Failed to set the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return -1;
    }

    // Enable Dynamixel Torque

    for (int i = 1; i <= NUM_MOTORS; i++) {

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel XL330-M288-T %03d has torque enabled \n", i);
        }

    }

    // Check voltage on all motors

    for (int i = 1; i <= NUM_MOTORS; i++) {
        uint8_t curV, minV, maxV;
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, i, 144, (uint8_t*)&curV);
        printf(" Current voltage on motor %03d is: %d\n", i, curV);
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, i, 34, (uint8_t*)&minV);
        printf(" Minimum voltage on motor %03d is: %d\n", i, minV);
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, i, 32, (uint8_t*)&maxV);
        printf(" Maximum voltage on motor %03d is: %d\n\n", i, maxV);

        if (curV <= minV || curV >= maxV) {
            printf("WARNING: Motor %03d is recieving a voltage that's out of range\n", i);
        }
    }


    int m[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
        m[i] = 0;
    }

    int step = 0;

    //Set arm to home position
    autoHome(packetHandler, portHandler);
    while (1)
    {

        //Wait until the Aruco Program sends the signal to set a new position or stop the program

        while (1) {

            //Check the file to see if it has recieved a new signal

            char inBuff;

            recvMsg(CommFile, inBuff);

            if (inBuff == 's')//If ArUcoRead sends a signal to set a new position
                break;

            if (inBuff == 'q') {//If ArUcoRead sends a signal to stop the program

                printf("Recieved signal to stop program\n");

                //clear comm file
                ofstream clearFile;
                clearFile.open(CommFile, ios::out | ios::trunc);
                clearFile.close();

                return 0;
            }
        }

        //Set a random position for all motors
        int lower_even = 3000, higher_even = 12500, lower_odd = 1500;
        for (int i = NUM_MOTORS; i >= 1; i--) {

                if (i % 2 == 0)//All even numbered motors
                {
                    m[i - 1] = lower_even + (rand() % (higher_even - lower_even));//Set position between 2000 and 12500
                }
        }

        //Motor number 4 has to be the lowest so the camera is able to see it
        //Set potition between 2000 and the other lowest motor
        if ((m[LESSER_MOTOR - 1] > m[HIGHER_MOTOR_1 - 1] || m[LESSER_MOTOR - 1] > m[HIGHER_MOTOR_2 - 1]) && m[HIGHER_MOTOR_1 - 1] < m[HIGHER_MOTOR_2 - 1])
            m[LESSER_MOTOR - 1] = m[HIGHER_MOTOR_1 - 1];
        else if ((m[LESSER_MOTOR - 1] > m[HIGHER_MOTOR_1 - 1] || m[LESSER_MOTOR - 1] > m[HIGHER_MOTOR_2 - 1]) && m[HIGHER_MOTOR_1 - 1] > m[HIGHER_MOTOR_2 - 1])
            m[LESSER_MOTOR - 1] = m[HIGHER_MOTOR_2 - 1];

        for (int i = 1; i < NUM_MOTORS; i++) {
            if (i % 2 != 0)//All odd numbered motors move clockwise to rise
                m[i - 1] = lower_odd + (rand() % (int)(0.7*m[i] - lower_odd));//m[i] cannot be grater than the next motor-500 and it must be greater than 
        }

        _sleep(500);
        for (int i = 1; i <= NUM_MOTORS; i++)
            printf("Position set for motor %03d is %d\n", i, m[i - 1]);



        // Enable Dynamixel Torque

        for (int i = 0; i <= NUM_MOTORS; i++) {

            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {

                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {

                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

        }

        //Set all motors positions

        for (int i = NUM_MOTORS - 1; i >= 0; i--) {

            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i + 1, ADDR_PRO_GOAL_POSITION, m[i], &dxl_error);

            //Check for errors

            if (dxl_comm_result != COMM_SUCCESS)
            {

                printf("%d", packetHandler->getTxRxResult(dxl_comm_result));
                uint8_t error;
                dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, i + 1, 70, (uint8_t*)&error);
                printf("Error on motor %d is %d\n", i + 1, error);

                //Send signal to OpenCV process that an error has occurred

                if (!sendMsg(CommFile, 'e'))
                    std::cout << "ERROR: Could not send message" << endl;

                return -1;
            }

            else if (dxl_error != 0)
            {

                printf("%d\n", packetHandler->getRxPacketError(dxl_error));
                uint8_t error;
                dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, i + 1, 70, (uint8_t*)&error);
                printf("Error on motor %d is %d\n", i + 1, error);

                //Send signal to OpenCV process that an error has occurred

                if (!sendMsg(CommFile, 'e')) {
                    std::cout << "ERROR: Could not send message" << endl;
                    return -1;
                }

                return -1;
            }
        }


        //Wait until all motors are set on their respective position

        for (int i = NUM_MOTORS; i >= 1; i--) {

            while (1) {

                dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, i, ADDR_PRO_MOVING_STATS, (uint8_t*)&MotorsSet);
                if ((MotorsSet % 2) != 0)
                    break;

                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&curpos);
                printf("Current position on Dynamixel %03d is: %d and is set to %d\n", i, curpos, m[i - 1]);
            }

            printf("\n\tMotor %03d reached set position\n", i);

        }
        printf("\n\n");
        _sleep(100);

        //Send signal to ArUcoRead to see if it can read an ArUco code
        if (!sendMsg(CommFile, 'r')) {
            std::cout << "ERROR: Could not send message" << endl;
            return -1;
        }

        //Wait for a signal from ArUcoRead to either write all data or set new position
        while (1) {

            char inBuff;
            recvMsg(CommFile, inBuff);

            if (inBuff == 's')//If ArUcoRead was not able to read the new set position and is asking for a different one
                break;

            if (inBuff == 'w') {//If ArUcoRead was able to read the new set position

                //Write all motors positions on the Data File

                stringstream strBuffer;

                for (int i = 1; i <= NUM_MOTORS; i++) {

                    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&curpos);
                    strBuffer << (int)curpos << "; ";
                }

                strBuffer << endl;

                long size = strBuffer.str().size() + 1;
                char* charBuffer = new char[size];
                strcpy_s(charBuffer, size, strBuffer.str().c_str());

                std::cout << "String: " << strBuffer.str() << endl << "Char: " << charBuffer << endl;

                DataFile.open("../../../../Data File.csv", ios::app);//Opens the .csv file and sets the cursor at the end
                DataFile.write(charBuffer, size);
                DataFile.close();

                break;
            }

            if (inBuff == 'q') {//If ArUcoRead sends a signal to stop the program

                printf("Recieved signal to stop program\n");
                return 0;
            }
        }

        //Send signal stating either all motors position have been written
        //or the signal to set a new position has been recieved

        if (!sendMsg(CommFile, 'o')) {
            std::cout << "ERROR: Could not send message" << endl;
            return -1;
        }
        cout << "Step: " << step << endl;
        step++;
    }

    // Disable Dynamixel Torque

    for (int i = 0; i <= NUM_MOTORS; i++) {

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {

            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {

            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

    }

    // Close port

    portHandler->closePort();

    return 0;
}