/*
 *
 * LiCAS External Control Interface (ECI) through UDP sockets - LiCAS_ECI_UDP.cpp
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
 * This class provides an UDP interface for sending control references to the LiCAS dual arm system
 * and for receiving feedback from the arms. Data packets are defined as a C-style data structures
 * that are sent/received through the UDP sockets, specifying the destination IP address and port,
 * and the reception port.
 *
 */

#include "LiCAS_ECI_UDP.h"



const uint8_t LiCAS_ECI_UDP::LiCAS_CONTROL_MODE_JOINT_POS = 1;	// Joint position control mode
const uint8_t LiCAS_ECI_UDP::LiCAS_CONTROL_MODE_JOINT_SPD = 2;	// Joint speed control mode
const uint8_t LiCAS_ECI_UDP::LiCAS_CONTROL_MODE_JOINT_TRQ = 3;	// Joint torque control mode
const uint8_t LiCAS_ECI_UDP::LiCAS_CONTROL_MODE_TCP_POS = 101;	// TCP position control mode
const uint8_t LiCAS_ECI_UDP::LiCAS_CONTROL_MODE_TCP_VEL = 102;	// TCP velocity control mode
const uint8_t LiCAS_ECI_UDP::LiCAS_CONTROL_MODE_TCP_FRC = 103;	// TCP force control mode

/*
 * Constructor
 *
 * Parameters:
 * 	(1) Name of the LiCAS interface (example: "LiCAS-A1")
 * */
LiCAS_ECI_UDP::LiCAS_ECI_UDP(const string &_LiCAS_Interface_Name)
{
	int k = 0;
	
	
	this->LiCAS_Interface_Name = _LiCAS_Interface_Name;
	
	// Init variables
	this->LiCAS_IP_Address = "";
	this->UDP_TxPort = -1;
	this->UDP_RxPort = -1;
	this->socketSender = -1;
	
	// Init time stamp
	gettimeofday(&tini, NULL);
	gettimeofday(&tact, NULL);
	elapsedTime = 0;
	
	this->flagFeedbackReceived = 0;
	this->flagTerminateThread = 0;
	this->flagRxThreadTerminated = 0;
	
	for(k = 0; k < NUM_ARM_JOINTS; k++)
	{
		this->qL[k] = 0;
		this->qR[k] = 0;
		this->dqL[k] = 0;
		this->dqR[k] = 0;
		this->tauL[k] = 0;
		this->tauR[k] = 0;
		this->pwmL[k] = 0;
		this->pwmR[k] = 0;
	}
	t_lastUpdate = 0;
	elapsedTimeLastUpdate = 0;
}


/*
 * Destructor
 * */
LiCAS_ECI_UDP::~LiCAS_ECI_UDP()
{
}


/*
 * Open the UDP socket interface for sending/receiving data to/from the LiCAS computer board.
 *
 * Parameters:
 * 	(1) IP address of the computer board executing the LiCAS control program
 *	(2) UDP port for sending the control references to the LiCAS control program
 *	(3) UDP port for receiving the feedback data packet from the LiCAS control program
 */
int LiCAS_ECI_UDP::openUDPInterface(const string &_LiCAS_IP_Address, int _UDP_TxPort, int _UDP_RxPort)
{
	int errorCode = 0;
	
	
	// Open the UDP socket for sending the control references to the LiCAS dual arm
	this->socketSender = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(socketSender < 0)
    {
    	errorCode = 1;
    	cout << endl << "ERROR: [in LiCAS_ECI_UDP::openUDPInterface] could not open socket." << endl;
   	}
   	else
	{
	   	host = gethostbyname(_LiCAS_IP_Address.c_str());
	    if(host == NULL)
		{
		    errorCode = 2;
			close(socketSender);
		    cout << "ERROR: [in LiCAS_ECI_UDP::openUDPInterface] could not get host by name." << endl;
		}
		else
		{
			// Set the address of the host
			bzero((char*)&addrHost, sizeof(struct sockaddr_in));
			this->addrHost.sin_family = AF_INET;
			bcopy((char*)host->h_addr, (char*)&addrHost.sin_addr.s_addr, host->h_length);
			this->addrHost.sin_port = htons(_UDP_TxPort);
		}
	}
	
	if(errorCode == 0)
	{
		// Copy the IP and UDP ports
		this->LiCAS_IP_Address = _LiCAS_IP_Address;
		this->UDP_TxPort = _UDP_TxPort;
		this->UDP_RxPort = _UDP_RxPort;
		
		// Init the thread for receiving the feedback data packet from the LiCAS dual arm
		udpRxThread = thread(&LiCAS_ECI_UDP::udpRxThreadFunction, this);
		udpRxThread.detach();
	}
	
	
	return errorCode;
}
	

/*
 * Send joint position references to the LiCAS dual arm.
 *
 * Parameters:
 * 	(1) Left arm joint position
 * 	(2) Right arm joint position
 * 	(3) Time for reaching the reference from current position
 */
int LiCAS_ECI_UDP::sendJointPositionRef(float * qLref, float * qRref, float playTime)
{
	LiCAS_CONTROL_REF_DATA_PACKET controlRefDataPacket;
	int bytesSent = 0;
	int k = 0;
	int errorCode = 0;
	
	
	// Set the fields of the data packet
	controlRefDataPacket.mode = this->LiCAS_CONTROL_MODE_JOINT_POS;
	controlRefDataPacket.playTime = playTime;
	for(k = 0; k < NUM_ARM_JOINTS; k++)
	{
		controlRefDataPacket.refLJ[k] = qLref[k];
		controlRefDataPacket.refRJ[k] = qRref[k];
	}
	controlRefDataPacket.timeStamp = this->getElapsedTime();
	
	
	// Send the control references data packet
	bytesSent = sendto(this->socketSender, (char*)&controlRefDataPacket, sizeof(LiCAS_CONTROL_REF_DATA_PACKET), 0, (struct sockaddr*)&addrHost, sizeof(struct sockaddr));
	if(bytesSent < 0)
	{
		errorCode = 1;
		cout << "ERROR: [in LiCAS_ECI_UDP::sendJointPositionRef] could not send data packet." << endl;
	}
	else if(bytesSent != sizeof(LiCAS_CONTROL_REF_DATA_PACKET))
	{
		errorCode = 1;
		cout << "ERROR: [in LiCAS_ECI_UDP::sendJointPositionRef] incorrect number packet." << endl;
	}
	
	
	return errorCode;
}

	
/*
 * Get the elapsed time since the creation of the interface instance
 */
float LiCAS_ECI_UDP::getElapsedTime()
{
	float t = 0;
	
	
	gettimeofday(&tact, NULL);
	elapsedTime = (tact.tv_sec - tini.tv_sec) + 1e-6*(tact.tv_usec - tini.tv_usec);
	
	t = (float)elapsedTime;
	
	
	return t;
}


void LiCAS_ECI_UDP::udpRxThreadFunction()
{
	LiCAS_FEEDBACK_DATA_PACKET * dataPacketFeedback;
	struct sockaddr_in addrReceiver;
	struct sockaddr_in addrSender;
	socklen_t addrLength;
	int socketReceiver = -1;
	int dataReceived = 0;
	char buffer[1024];
	
	ofstream LiCAS_DataLogFile;
	
	int errorCode = 0;
	int k = 0;
	
	
	// Open the socket in datagram mode
	socketReceiver = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(socketReceiver < 0) 
	{
		errorCode = 1;
		cout << endl << "ERROR: [in LiCAS_ECI_UDP::udpRxThreadFunction] could not open socket." << endl;
	}
	else
	{
		// Set listenning address (any) and port
		bzero((char*)&addrReceiver, sizeof(struct sockaddr_in));
		addrReceiver.sin_family = AF_INET;
		addrReceiver.sin_addr.s_addr = INADDR_ANY;
		addrReceiver.sin_port = htons(this->UDP_RxPort);
	
		// Associates the address to the socket
		if(bind(socketReceiver, (struct sockaddr*)&addrReceiver, sizeof(addrReceiver)) < 0)
		{
			errorCode = 2;
			close(socketReceiver);
			cout << endl << "ERROR: [in LiCAS_ECI_UDP::udpRxThreadFunction] could not associate address to socket." << endl;
		}
		else
		{
			// Set the socket as non blocking
			fcntl(socketReceiver, F_SETFL, O_NONBLOCK);
		}
	}
	
	// Open log data file
	if(errorCode == 0)
	{
		LiCAS_DataLogFile.open("LiCAS_DataLog.txt");
	}

	/******************************** THREAD LOOP START ********************************/

	while(errorCode == 0 && flagTerminateThread == 0)
	{
		dataReceived = recvfrom(socketReceiver, buffer, 1023, 0, (struct sockaddr*)&addrSender, &addrLength);
		if(dataReceived == sizeof(LiCAS_FEEDBACK_DATA_PACKET))
		{
			// Get pointer to data in the structure from the buffer pointer
			dataPacketFeedback = (LiCAS_FEEDBACK_DATA_PACKET*)buffer;
			
			// Copy the received feedback on the public variables
			for(k = 0; k < 3; k++)
			{
				this->pL[k] = dataPacketFeedback->pL[k];
				this->pR[k] = dataPacketFeedback->pR[k];
			}
			for(k = 0; k < NUM_ARM_JOINTS; k++)
			{
				this->qL[k] = dataPacketFeedback->qL[k];
				this->qR[k] = dataPacketFeedback->qR[k];
				this->dqL[k] = dataPacketFeedback->dqL[k];
				this->dqR[k] = dataPacketFeedback->dqR[k];
				this->tauL[k] = dataPacketFeedback->tauL[k];
				this->tauR[k] = dataPacketFeedback->tauR[k];
				this->pwmL[k] = dataPacketFeedback->pwmL[k];
				this->pwmR[k] = dataPacketFeedback->pwmR[k];
				
				printf("LEFT ARM Cartesian Position: {%.1f, %.1f, %.1f} [cm]\n", 100*pL[0], 100*pL[1], 100*pL[2]);
				printf("LEFT ARM Joint Position: {%.1f, %.1f, %.1f, %.1f} [deg]\n", qL[0], qL[1], qL[2], qL[3]);
				printf("LEFT ARM Joint Velocity: {%.1f, %.1f, %.1f, %.1f} [deg/s]\n", dqL[0], dqL[1], dqL[2], dqL[3]);
				printf("LEFT ARM Joint PWM: {%.1f, %.1f, %.1f, %.1f} \n", pwmL[0], pwmL[1], pwmL[2], pwmL[3]);
				printf("\n");
				
				printf("RIGHT ARM Cartesian Position: {%.1f, %.1f, %.1f} [cm]\n", 100*pR[0], 100*pR[1], 100*pR[2]);
				printf("RIGHT ARM Joint Position: {%.1f, %.1f, %.1f, %.1f} [deg]\n", qR[0], qR[1], qR[2], qR[3]);
				printf("RIGHT ARM Joint Velocity: {%.1f, %.1f, %.1f, %.1f} [deg/s]\n", dqR[0], dqR[1], dqR[2], dqR[3]);
				printf("RIGHT ARM Joint PWM: {%.1f, %.1f, %.1f, %.1f} \n", pwmR[0], pwmR[1], pwmR[2], pwmR[3]);
				printf("\n");
				printf("---\n");
			}
				
		
			// Save data on log file
			LiCAS_DataLogFile << getElapsedTime() << "\t";
			for(k = 0; k < 3; k++)
				LiCAS_DataLogFile << pL[k] << "\t";
			for(k = 0; k < 3; k++)
				LiCAS_DataLogFile << pR[k] << "\t";
			for(k = 0; k < NUM_ARM_JOINTS; k++)
				LiCAS_DataLogFile << qL[k] << "\t";
			for(k = 0; k < NUM_ARM_JOINTS; k++)
				LiCAS_DataLogFile << qR[k] << "\t";
			for(k = 0; k < NUM_ARM_JOINTS; k++)
				LiCAS_DataLogFile << dqL[k] << "\t";
			for(k = 0; k < NUM_ARM_JOINTS; k++)
				LiCAS_DataLogFile << dqR[k] << "\t";
			for(k = 0; k < NUM_ARM_JOINTS; k++)
				LiCAS_DataLogFile << tauL[k] << "\t";
			for(k = 0; k < NUM_ARM_JOINTS; k++)
				LiCAS_DataLogFile << tauR[k] << "\t";
			for(k = 0; k < NUM_ARM_JOINTS; k++)
				LiCAS_DataLogFile << pwmL[k] << "\t";
			for(k = 0; k < NUM_ARM_JOINTS; k++)
				LiCAS_DataLogFile << pwmR[k] << "\t";
			LiCAS_DataLogFile << endl;
		
			
			elapsedTimeLastUpdate = getElapsedTime() - t_lastUpdate;
			t_lastUpdate = getElapsedTime();
			
			// Set the feedback received float
			this->flagFeedbackReceived = 1;
		}
		
		// Wait 10 ms
		usleep(10000);
	}
	
	/******************************** THREAD LOOP END ********************************/
	
	// Close log file
	LiCAS_DataLogFile.close();
	
	// Close the socket
	if(errorCode == 0)
		close(socketReceiver);
	this->flagRxThreadTerminated = 1;
}

	
/*
 * Close the UDP socket interface
 */
int LiCAS_ECI_UDP::closeInterface()
{
	int errorCode = 0;
	float timer = 0;


	this->flagTerminateThread = 1;
	usleep(10000);	// Waits 10 ms to termiante thread

	// Close sender socket
	close(this->socketSender);
	this->socketSender = -1;

	cout << "Waiting reception thread termination..." << endl;
	while(this->flagRxThreadTerminated == 0 && timer < 1)
	{
		usleep(10000);
		timer += 0.01;
	}
	
	if(timer >= 1)
	{
		errorCode = 1;
		cout << "ERROR [in LiCAS_ECI_UDP::closeInterface]: could not terminate reception thread." << endl;
	}
	else
		cout << "LiCAS External Control Interface UDP terminated correctly." << endl;

	
	return errorCode;
}

