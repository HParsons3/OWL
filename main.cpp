/*
Phil Culverhouse Oct 2016 (c) Plymouth University
James Rogers Jan 2020     (c) Plymouth University
This demo code will move eye and neck servos with kepresses.
Use this code as a base for your assignment.
*/

#include <iostream>
#include "math.h"
#include <fstream>
#include <string>
#include <sys/types.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

using namespace std;
using namespace cv;

double pi = 3.14159265359;

int main(int argc, char *argv[])
{
    //Setup TCP coms
    ostringstream CMDstream; // string packet
    string CMD;
    string PiADDR = "10.0.0.10";
    int PORT=12345;
    SOCKET u_sock = OwlCommsInit(PORT, PiADDR);

    //Set servo positions to their center-points
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    // move servos to centre of field
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
    string RxPacket= OwlSendPacket (u_sock, CMD.c_str());

    Mat Frame, Left, Right;

    //Open video feed
    string source = "http://10.0.0.10:8080/stream/video.mjpeg";
    VideoCapture cap (source);
    if (!cap.isOpened())
    {
        cout  << "Could not open the input video: " << source << endl;
        return -1;
    }
    int Direction = 1;
    int actionChoice = 0;
    int TL;
    int TR;
    int rollSection = 0;
    int crossDirection = 1;
    //main program loop
    while (1){
        if (!cap.read(Frame))
        {
            cout  << "Could not open the input video: " << source << endl;
            break;
        }

        //flip input image as it comes in reversed
        Mat FrameFlpd;
        flip(Frame,FrameFlpd,1);

        // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
        Left= FrameFlpd(Rect(0, 0, 640, 480)); // using a rectangle
        Right=FrameFlpd(Rect(640, 0, 640, 480)); // using a rectangle

        //Draw a circle in the middle of the left and right image (usefull for aligning both cameras)
        circle(Left,Point(Left.size().width/2,Left.size().height/2),10,Scalar(255,255,255),1);
        circle(Right,Point(Right.size().width/2,Right.size().height/2),10,Scalar(255,255,255),1);

        //Display left and right images
        imshow("Left",Left);
        imshow("Right", Right);
        waitKey(10);

        //Read keypress and move the corresponding motor
        int key = waitKey(10);
        switch (key){
//        case 'w': //up
//            Ry=Ry+5;
//            break;
        case 's'://down
            actionChoice = 2;
            break;
        case 'a'://left
            actionChoice = 1;
            break;
        case 'd'://right
            actionChoice = 3;
            break;
        case 'f':
            actionChoice = 4;
            break;
//        case 'i': //up
//            Ly=Ly-5;
//            break;
//        case 'k'://down
//            Ly=Ly+5;
//            break;
//        case 'j'://left
//            Lx=Lx-5;
//            break;
//        case 'l'://right
//            Lx=Lx+5;
//            break;
        case 'e'://right
            actionChoice = 0;
            break;
//        case 'q'://left
//            Neck=Neck-5;
//            break;
        }
        switch (actionChoice){
          case 0:
            //Default eye location
            Rx=1445;
            Ry=1520;
            Lx=1450;
            Ly=1460;
            break;
          case 1:
            //Horizontal Scan
            Rx = Rx + Direction*5; //Modify x value by 5 or -5, depending on which direction
            Lx = Lx + Direction*5;
            CMDstream.str("");
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();
            RxPacket= OwlSendPacket (u_sock, CMD.c_str()); //Update
            if((Rx == 1870 && Direction == 1) || (Rx == 1200 && Direction == -1)) {
                Direction = -Direction; //If the eye reaches the max or min value, swap direction
            }
            Sleep(10);
            break;
          case 2:
            //Chameleon eyes
            TL = rand() % 100; //Random percentage
            TR = rand() % 100;
            if (TL >= 97) { //3% chance every iteration to choose new location
            Lx = (rand() % 670) + 1180; //Random angle values from min to max, using formula [(Random from 0 to (Max-min)) + min]
            Ly = (rand() % 820) + 1180; 
            CMDstream.str("");
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();
            RxPacket= OwlSendPacket (u_sock, CMD.c_str()); //Update 
            }
            if (TR >= 97) {
            Rx = (rand() % 690) + 1200; //Same formula as left eye
            Ry = (rand() % 880) + 1120;
            CMDstream.str("");
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();
            RxPacket= OwlSendPacket (u_sock, CMD.c_str()); //Update
            }
            break;
          case 3:
            //Roll
            Rx = round(1545+(sin(5*rollSection*(pi/180))*335)); //Median value + (sin(desired angle in radians)*((Max-Min)/2)
            Lx = round(1515+(sin(5*rollSection*(pi/180))*335)); //This gives a sinusoidal movement to x axis
            Ry = round(1760+(sin((5*rollSection+90)*(pi/180))*240)); //Added 90 degrees to desired angle of y, otherwise we get a diagonal instead of circular
            Ly = round(1320+(sin((5*rollSection+90)*(pi/180))*-140)); //*-140 because the left eye's y axis is reversed
//            CMDstream.str("");
//            CMDstream.clear();
//            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
//            CMD = CMDstream.str();
//            RxPacket= OwlSendPacket (u_sock, CMD.c_str());
            rollSection++; //Increment angle
            Sleep(5);
            break;
          case 4:
            //Cross-eyed
            Ry = 1520; //Set y axis to medium
            Ly = 1460;
            if((Rx <= RxLm) || (Rx >= RxC)) {
                crossDirection = -crossDirection; //If the eye reaches the set min or max (from leftmost to middle), reverse direction
            } 
            Rx = Rx + 20*crossDirection; //Move in the desired direction
            Lx = Lx - 20*crossDirection; //opposite direction, as we are looking for cross-eyes, not looking the same way.
            Sleep(5);

//            Rx = RxLm;
//            Lx = LxRm;
//            CMDstream.str("");
//            CMDstream.clear();
//            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
//            CMD = CMDstream.str();
//            RxPacket= OwlSendPacket (u_sock, CMD.c_str());
//            Sleep(250);
//            Rx = 1445;
//            Lx = 1450;
//            CMDstream.str("");
//            CMDstream.clear();
//            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
//            CMD = CMDstream.str();
//            RxPacket= OwlSendPacket (u_sock, CMD.c_str());
//            Sleep(250);
            break;
        }

        //Send new motor positions to the owl servos
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        RxPacket= OwlSendPacket (u_sock, CMD.c_str());

    } // END cursor control loop

    // close windows down
    destroyAllWindows();


#ifdef _WIN32
    RxPacket= OwlSendPacket (u_sock, CMD.c_str());
    closesocket(u_sock);
#else
    OwlSendPacket (clientSock, CMD.c_str());
    close(clientSock);
#endif
    exit(0); // exit here for servo testing only
}
