/*
 *
 * LiCAS External Control Interface (ECI) - Main.cpp
 *
 * Copyright (c) 2025 Alejandro Suarez, asuarezfm@us.es
 *
 * LiCAS Robotic Arms Project: Lightweight and Compliant Anthropomorphic Dual Arm System
 *
 * Instagram: licas_ra
 * LinkedIn: LiCAS Robotic Arms
 *
 * Date: November 2025
 *
 * This program makes use of the LiCAS External Control Interface (ECI) implementd through
 * UDP sockets for sending control references and receiving feedback to the LiCAS dual arm.
 * The references generated here are sent to the LiCAS Control Program executed in a computer
 * board (or in localhost) accessed through its IP address and UDP port. The LiCAS ECI takes
 * as input argument the IP address, transmitting UDP port and receiving UDP port for sending
 * control commands and receiving feedback from the arms, respectively. The dual arm can be
 * either a physical robot or a simulated one.
 *
 */


// Standard library
#include <iostream>
#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>


// Specific library
#include "../LiCAS_ECI_UDP/LiCAS_ECI_UDP.h"



// Namespaces
using namespace std;


int main(int argc, char ** argv)
{
	LiCAS_ECI_UDP * licas_eci = NULL;
	string cmd;
	double t = 0;
	int errorCode = 0;
	
	
	cout << "__________________________________________" << endl;
	cout << "LiCAS External Control Interface (ECI) UDP" << endl;
	cout << "Author: Alejandro Suarez, asuarezfm@us.es" << endl;
	cout << "LiCAS Robotic Arms Initiative, #licas_ra" << endl;
	cout << "__________________________________________" << endl;
	cout << endl;
	
	if(argc != 4)
	{
		cout << "ERROR [in main]: invalid number of arguments." << endl;
		cout << "Specify IP address, UDP Tx port and UDP Rx port." << endl;
		cout << "Example: ./LiCAS_ECI 10.43.0.110 23000 24000" << endl;
		cout << endl;
	}
	else
	{
		// Create the instance to the LiCAS External Control Interface
		licas_eci = new LiCAS_ECI_UDP("LiCAS_A1_Interface");
	
		// Open the UDP socket interface
		errorCode = licas_eci->openUDPInterface(argv[1], atoi(argv[2]), atoi(argv[3]));
		
		if(errorCode != 0)
			cout << "ERROR [in main]: could not open LiCAS ECI" << endl;
		else
		{
			// Joint position references
			float qLref[NUM_ARM_JOINTS];
			float qRref[NUM_ARM_JOINTS];
			double AL[NUM_ARM_JOINTS] = {-30, 10, -45, -60};
			double AR[NUM_ARM_JOINTS] = {-30, -10, 45, -60};
			float f = 0.25;
			float playTime = 0.25;
			
			// Generate a sinusoidal joint position reference for 10 seconds
			while(t < 10.0)
			{
				for(int k = 0; k < NUM_ARM_JOINTS; k++)
				{
					qLref[k] = AL[k]*sin(6.28*0.25*t);
					qLref[k] = AR[k]*sin(6.28*0.25*t);
				}
				
				// Send the joint reference through the external control interface
				licas_eci->sendJointPositionRef(qLref, qRref, playTime);
				
				// Wait 20 ms = 50 Hz rate
				usleep(20000);
				
				// Update time stamp
				t += 0.02;
			}
			
			// Close interface
			licas_eci->closeInterface();
		}
	}
		
	
	return errorCode;
}






