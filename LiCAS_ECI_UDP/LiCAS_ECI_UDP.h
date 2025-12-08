/*
 *
 * LiCAS External Control Interface (ECI) through UDP sockets - LiCAS_ECI_UDP.h
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

#ifndef LICAS_ECI_UDP_H_
#define LICAS_ECI_UDP_H_


// Standard library
#include <iostream>
#include <thread>
#include <fstream>
#include <string>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>


// Constant definition
#define NUM_ARM_JOINTS	4	// Number of joints of each arm


using namespace std;


class LiCAS_ECI_UDP
{
public:

	/***************** PUBLIC VARIABLES *****************/
	
	float pL[3];					// Cartesian position of left TCP in [m]
	float pR[3];					// Cartesian position of right TCP in [m]
	float qL[NUM_ARM_JOINTS];		// Joint position left arm in [rad]
	float qR[NUM_ARM_JOINTS];		// Joint position right arm in [rad]
	float dqL[NUM_ARM_JOINTS];		// Joint speed left arm in [rad/s]
	float dqR[NUM_ARM_JOINTS];		// Joint speed right arm in [rad/s]
	float tauL[NUM_ARM_JOINTS];		// Joint torque left arm in [Nm]
	float tauR[NUM_ARM_JOINTS];		// Joint torque right arm in [Nm]
	float pwmL[NUM_ARM_JOINTS];		// Joint PWM left arm in [-1, 1]
	float pwmR[NUM_ARM_JOINTS];		// Joint PWM right arm in [-1, 1]
	float t_lastUpdate;				// Instance time since last update
	float elapsedTimeLastUpdate;	// Elapsed time since last update
	
	
	/***************** PUBLIC METHODS *****************/
	
	/*
	 * Constructor
	 *
	 * Parameters:
	 * 	(1) Name of the LiCAS interface (example: "LiCAS-A1")
	 * */
	LiCAS_ECI_UDP(const string &_LiCAS_Interface_Name);


	/*
	 * Destructor
	 * */
	virtual ~LiCAS_ECI_UDP();
	
	
	/*
	 * Open the UDP socket interface for sending/receiving data to/from the LiCAS computer board.
	 *
	 * Parameters:
	 * 	(1) IP address of the computer board executing the LiCAS control program
	 *	(2) UDP port for sending the control references to the LiCAS control program
	 *	(3) UDP port for receiving the feedback data packet from the LiCAS control program
	 */
	int openUDPInterface(const string &_LiCAS_IP_Address, int _UDP_TxPort, int _UDP_RxPort);
	
	
	/*
	 * Send joint position references to the LiCAS dual arm.
	 *
	 * Parameters:
	 * 	(1) Left arm joint position
	 * 	(2) Right arm joint position
	 * 	(3) Time for reaching the reference from current position
	 */
	int sendJointPositionRef(float * qLref, float * qRref, float playTime);
	
	
	/*
	 * Send TCP (tool center point) position references to the LiCAS dual arm.
	 *
	 * Parameters:
	 * 	(1) Left arm TCP position reference
	 * 	(2) Right arm TCP position reference
	 * 	(3) Time for reaching the reference from current position
	 */
	int sendTCPPositionRef(float * pLref, float * pRref, float playTime);
	
	
	/*
	 * Get the elapsed time since the creation of the interface instance
	 */
	float getElapsedTime();
	
	
	/*
	 * Close the UDP socket interface.
	 */
	int closeInterface();
	

private:

	// NOTES
	// -----
	// Joint position in radians
	// Joint speed in rad/s
	// Joint torque in Nm
	// PWM (pulse width modulation) in [-1, 1] range
	// TCP position in m w.r.t. shoulder base joint
	// TCP velocity in m/s w.r.t. shoulder base joint
	// TCP force in N w.r.t. shoulder base joint

	static const uint8_t LiCAS_CONTROL_MODE_JOINT_POS;	// Joint position control mode
	static const uint8_t LiCAS_CONTROL_MODE_JOINT_SPD;	// Joint speed control mode
	static const uint8_t LiCAS_CONTROL_MODE_JOINT_TRQ;	// Joint torque control mode
	static const uint8_t LiCAS_CONTROL_MODE_TCP_POS;	// TCP position control mode
	static const uint8_t LiCAS_CONTROL_MODE_TCP_VEL;	// TCP velocity control mode
	static const uint8_t LiCAS_CONTROL_MODE_TCP_FRC;	// TCP force control mode
	
	typedef struct
	{
		uint8_t mode;
		float playTime;
		float refLTCP[3];					// Reference value left arm TCP
		float refRTCP[3];					// Reference value right arm TCP
		float refLJ[NUM_ARM_JOINTS];		// Reference value left arm joints
		float refRJ[NUM_ARM_JOINTS];		// Reference value right arm joints
		float timeStamp;
	} __attribute__((packed)) LiCAS_CONTROL_REF_DATA_PACKET;
	
	
	typedef struct
	{
		uint8_t packetID;
		float pL[3];				// Cartesian position of left TCP in [m]
		float pR[3];				// Cartesian position of right TCP in [m]
		float qL[NUM_ARM_JOINTS];	// Joint position left arm in [rad]
		float qR[NUM_ARM_JOINTS];	// Joint position right arm in [rad]
		float dqL[NUM_ARM_JOINTS];	// Joint speed left arm in [rad/s]
		float dqR[NUM_ARM_JOINTS];	// Joint speed right arm in [rad/s]]
		float tauL[NUM_ARM_JOINTS];	// Joint torque left arm in [Nm]
		float tauR[NUM_ARM_JOINTS];	// Joint torque right arm in [Nm]
		float pwmL[NUM_ARM_JOINTS];	// PWM left arm joints in [-1, 1]
		float pwmR[NUM_ARM_JOINTS];	// PWM right arm joints in [-1, 1]
	} __attribute__((packed)) LiCAS_FEEDBACK_DATA_PACKET;
	

	/***************** PRIVATE VARIABLES *****************/
	string LiCAS_Interface_Name;
	
	thread udpRxThread;
	
	struct sockaddr_in addrHost;
    struct hostent * host;
	string LiCAS_IP_Address;
	int UDP_TxPort;
	int UDP_RxPort;
	int socketSender;
	
	struct timeval tini;
	struct timeval tact;
	double elapsedTime;
	
	int flagFeedbackReceived;
	int flagTerminateThread;
	int flagRxThreadTerminated;
	
	
	/***************** PRIVATE METHODS *****************/
	
	void udpRxThreadFunction();
	

};

#endif


