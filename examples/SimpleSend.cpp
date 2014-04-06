// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Confidential and not for redistribution. See LICENSE.txt.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

//For Mouse Controller
#include <windows.h>
#include "wtypes.h"
using namespace std;

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo.hpp>

// OSC Libs
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

// OSC Vars
#define ADDRESS "127.0.0.1"
#define PORT 7000
#define OUTPUT_BUFFER_SIZE 1024

// Get the horizontal and vertical screen sizes in pixel
void GetDesktopResolution(int& horizontal, int& vertical)
{
   RECT desktop;
   // Get a handle to the desktop window
   const HWND hDesktop = GetDesktopWindow();
   // Get the size of screen to the variable desktop
   GetWindowRect(hDesktop, &desktop);
   // The top left corner will have coordinates (0,0)
   // and the bottom right corner will have coordinates
   // (horizontal, vertical)
   horizontal = desktop.right;
   vertical = desktop.bottom;
}

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
    }
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;

        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(2.0f * (quat.w() * quat.y() - quat.z() * quat.x()));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                        1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

        // Convert the floating point angles in radians to a scale from 0 to 360. <= Used to be 0-20... 
        //roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 360);
        //pitch_w = static_cast<int>((pitch + (float)M_PI)/(M_PI * 2.0f) * 360);
        //yaw_w = static_cast<int>(((float)M_PI - yaw)/(M_PI * 2.0f) * 360);

        roll_w = roll;
        pitch_w = pitch;
        yaw_w = yaw;
    }

    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;

        // Vibrate the Myo whenever we've detected that the user has made a fist.
        //if (pose == myo::Pose::fist) {
        //    myo->vibrate(myo::Myo::VibrationMedium);
        //}

    }

    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.

	
	//Niko's Getters for Rotation of Myo. Rotations are 0-20; & going clock wise they repeat ((12 O'Clock) 0-20:(6 O'Clock):20-0 (12 O'Clock))
	float get_roll()
	{
		return roll_w;
	}

	float get_pitch()
	{
		return pitch_w;
	}

	float get_yaw()
	{
		return yaw_w;
	}

	//Return Pose Int: fist=>1; none=>0
	int get_pose()
	{
		std::string poseString = currentPose.toString();

		if(poseString == "fist")
		{
			return 1;
		}
		
		else if(poseString == "fingers_spread")
		{
			return 2;
		}

		else if(poseString == "twist_in")
		{
			return 3;
		}
		
		else if(poseString == "wave_in")
		{
			return 4;
		}
		
		else if(poseString == "wave_out")
		{
			return 5;
		}

		else
		{
			return 0;
		}
	}

    // We define this function to print the current values that were updated by the on...() functions above.
	/*
    void print()
    {
        // Clear the current line
        std::cout << '\r';

        // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
        // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
        // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
        std::string poseString = currentPose.toString();

        // Output the current values
        std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
                  << '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
                  << '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']'
                  << '[' << poseString << std::string(16 - poseString.size(), ' ') << ']'
                  << std::flush;
    }
	*/

    // These values are set by onOrientationData() and onPose() above.
    float roll_w, pitch_w, yaw_w;
    myo::Pose currentPose;
};

int main(int argc, char** argv)
{
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {

    // First, we create a Hub. The Hub provides access to one or more Myos.
    myo::Hub hub;

    std::cout << "Attempting to find a Myo..." << std::endl;

    // Next, we try to find a Myo (any Myo) that's nearby and connect to it. waitForAnyMyo() takes a timeout
    // value in milliseconds. In this case we will try to find a Myo for 10 seconds, and if that fails, the function
    // will return a null pointer.
    myo::Myo* myo = hub.waitForAnyMyo(10000);

    // If waitForAnyMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
    if (!myo) {
        throw std::runtime_error("Unable to find a Myo!");
    }

    // We've found a Myo, let's output its MAC address.
    std::cout << "Connected to " << myo->macAddressAsString() << "." << std::endl << std::endl;

    // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
    DataCollector collector;

    // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
    // Hub::run() to send events to all registered device listeners.
    hub.addListener(&collector);

	// OSC Init of Socket & Buffer
	UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );
    
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );

	//Get Screen Size for mouse controller
	int horRez = 0;
	int verRez = 0;

	GetDesktopResolution(horRez, verRez);

	//Myo Gesture Holders
	int prev_pose = 0;

    // Finally we enter our main loop.
    while (1) {
        // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
        // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
        hub.run(1000/20);

		//This sends roll, pitch, yaw, and the pose packaged in that order
		///*
		p << osc::BeginBundleImmediate
        << osc::BeginMessage( "/test1" ) << collector.get_roll() << collector.get_pitch() << collector.get_yaw() << collector.get_pose() << osc::EndMessage
        << osc::EndBundle;    
		transmitSocket.Send( p.Data(), p.Size() );

		p.Clear();
		//*/
		/*
		int mouse_x = -(int)((horRez/2)*cos(collector.get_yaw() - (float)M_PI/2)*2)+horRez/2;
		int mouse_y = (int)((verRez/2)*sin(collector.get_pitch())*3)+verRez/2;

		SetCursorPos(mouse_x,mouse_y);
		
		cout<<collector.get_pitch()<<endl;
		cout<<collector.get_yaw()<<endl;

		if(collector.get_pose() == 1 && prev_pose != 1)
		{
			 mouse_event(MOUSEEVENTF_LEFTDOWN,mouse_x,mouse_y,0,0);
		}
		else if(collector.get_pose() != 1 && prev_pose == 1)
		{
			 mouse_event(MOUSEEVENTF_LEFTUP,mouse_x,mouse_y,0,0);
		}

		prev_pose = collector.get_pose();
		*/
    }

    // If a standard exception occurred, we print out its message and exit.
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}

//OSCPACK ORIGONAL CODE

/* 
    Simple example of sending an OSC message using oscpack.
*/
 /*
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

#include <iostream>
using namespace std;

#define ADDRESS "127.0.0.1"
#define PORT 7000

#define OUTPUT_BUFFER_SIZE 1024

int main(int argc, char* argv[])
{
    (void) argc; // suppress unused parameter warnings
    (void) argv; // suppress unused parameter warnings

    UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );
    
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );

	while(true)
	{
    p << osc::BeginBundleImmediate
        << osc::BeginMessage( "/test1" ) << 24 << osc::EndMessage
        << osc::EndBundle;
    
    transmitSocket.Send( p.Data(), p.Size() );
	}
}
*/


//You'll need to also link to the lib and header files in Myo and add the appropriate dills to the source folder? Or is it the system dlls?