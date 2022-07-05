/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of GraphGNSSLib.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 * Function: decode the RINEX file, output GNSS raw measurements via ros topics
 * Date: 2020/11/27
 *******************************************************/

#include <stdarg.h>
#include <ros/ros.h>
#include <stdio.h>
#include <assert.h>
#include "../RTKLIB/src/rtklib.h"
#include "myParameters.h"

#define TRACE
// extern void postposRegisterPub(ros::NodeHandle &n);
// extern void rtkposRegisterPub(ros::NodeHandle &n);
// extern void pntposRegisterPub(ros::NodeHandle &n);

bool paramVerbose;
int paramWindowSize;
int paramMaxIterations;
int paramMaxIterationsEnd;
bool paramOptimizeBiasesAgain;
bool paramOptimizeBiasesAgainEnd;
int paramLaserInform;
int paramLaserTimeOffset;
bool paramFilterGPS;
int paramMaxGPSSpeed;
int paramMaxGPSVertSpeed;
int paramMaxAltToDstPct;
int paramSkipGPSPoses;
int paramDecimation;
int paramPosesToProcess;
int paramDopplerInformFactor;
int paramVelToDopplerRatiox10;
std::string paramSlamOdometryPath; 

int main(int argc, char **argv)
{
	// my_g2o_main();
    ros::init(argc, argv, "gals_node");
	ros::NodeHandle nh("~");
	ROS_INFO("gals_node started.");

	/* get setup parameters */
	int mode, nf, soltype;
	std::string roverMeasureFile, navigationFile1, navigationFile2, navigationFile3, navigationFile4, navigationFile5;
	std::string out_folder;
	nh.param("mode",   mode, 2);
	nh.param("nf",     nf, 3);
	nh.param("soltype",soltype, 2);
	ros::param::get("roverMeasureFile", roverMeasureFile);
	ros::param::get("navigationFile1", navigationFile1);
	ros::param::get("navigationFile2", navigationFile2);
	ros::param::get("navigationFile3", navigationFile3);
	ros::param::get("navigationFile4", navigationFile4);
	ros::param::get("navigationFile5", navigationFile5);

	ros::param::get("out_folder", out_folder);

	// Read optimization parameters
	nh.param("verbose",paramVerbose, false);
	nh.param("windowSize",paramWindowSize, 20);
	nh.param("maxIterations",paramMaxIterations, 50);
	nh.param("maxIterationsEnd",paramMaxIterationsEnd, 10);
	nh.param("optimizeBiasesAgain",paramOptimizeBiasesAgain, true);
	nh.param("optimizeBiasesAgainEnd",paramOptimizeBiasesAgainEnd, false);
	nh.param("laserInform", paramLaserInform, 10);
	nh.param("laserTimeOffset", paramLaserTimeOffset, 10);
	nh.param("filterGPS", paramFilterGPS, false);
	nh.param("maxGPSSpeed", paramMaxGPSSpeed, 20);
	nh.param("maxGPSVertSpeed", paramMaxGPSVertSpeed, 5);
	nh.param("maxAltToDstPct", paramMaxAltToDstPct, 20);
	nh.param("skipGPSPoses", paramSkipGPSPoses, 20);
	nh.param("decimation", paramDecimation, 1);
	if (!nh.param("posesToProcess", paramPosesToProcess, 1)){
		ROS_INFO("\033[1;31m Parameter posesToProcess was not set \033[0m");
		return 0;
	}
	nh.param("dopplerInformFactor", paramDopplerInformFactor, 1);
	nh.param("velToDopplerRatiox10", paramVelToDopplerRatiox10, 20);
	nh.param<std::string>("slamOdometryPath", paramSlamOdometryPath, "/");

	/* flag for state */
    int n=0,i,stat;

	/* processing time setting */
	double ti=0.0;						// processing interval  (s) (0:all)
	double tu=0.0;						// unit time (s) (0:all)
	gtime_t ts={0},te={0};
	ts.time=0;							// start time (0:all)
	te.time=0;							// end time (0:all)

	/* options */
	prcopt_t prcopt = prcopt_default;	// processing option
	solopt_t solopt = solopt_default;	// output solution option
	filopt_t filopt = {""};	            // file option
	prcopt.mode = mode;			// Kinematic RTK
	// prcopt.mode = PMODE_SINGLE;			// SPP
	prcopt.navsys = SYS_ALL;              // use all satellites system
	prcopt.nf = nf;						// frequency (1:L1,2:L1+L2,3:L1+L2+L5) 
	prcopt.soltype = soltype;					// 0:forward,1:backward,2:combined
	prcopt.elmin = 15.0*D2R;				// elevation mask (rad)	
	prcopt.tidecorr = 0;					// earth tide correction (0:off,1-:on) 
	prcopt.posopt[4] = 0;               // use RAIM FDE (qmo)  1
	prcopt.tropopt = TROPOPT_SAAS;        // troposphere option: Saastamoinen model
	prcopt.ionoopt = IONOOPT_BRDC;		// ionosphere option: Broad cast
	prcopt.sateph = EPHOPT_BRDC;			// ephemeris option: broadcast ephemeris
	// prcopt.dynamics = 0;
	// prcopt.niter = 1;
	prcopt.modear = 3;					// AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold)
	prcopt.dynamics = 0; 	// Works for DGPS, not for single positioning
	solopt.outopt = 1;					// output processing options (0:no,1:yes)
	solopt.timef = 0;						// time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s)
	solopt.timeu = 3;						// time digits under decimal point
	solopt.sep[0] = ',';					// field separator
	solopt.sstat= 0;						// solution statistics level (0:off,1:states,2:residuals)
	solopt.trace = 2;						// debug trace level (0:off,1-5:debug)
	solopt.sstat = 1;						// get the solution file
	solopt.posf = SOLF_LLH;
	solopt.height = 0;

	// solopt.maxsolstd = 0.1;
	char *rov="",*base="";
	char infile_[10][1024]={""}, *infile[10];
	char outfile[1024];

	/* set input files */
	for (i=0;i<10;i++) infile[i]=infile_[i];

	strcpy(infile[n++],strdup(roverMeasureFile.c_str()));
	strcpy(infile[n++],strdup(navigationFile1.c_str()));
	strcpy(infile[n++],strdup(navigationFile2.c_str()));
	strcpy(infile[n++],strdup(navigationFile3.c_str()));
	strcpy(infile[n++],strdup(navigationFile4.c_str()));
	strcpy(infile[n++],strdup(navigationFile5.c_str()));

	/* if you use the RTK mode, specify the position of the station (only used by RTKLIB)
	 * following is an example position of the base HKSC in Hong Kong */
	prcopt.refpos = 3;

	/* set output files */
	strcpy(outfile, strdup(out_folder.c_str()));

	/* decode the RINEX file positioning */
	stat = postpos(ts, te, ti, tu, &prcopt, &solopt, &filopt, infile, n, outfile, rov, base);

	printf("\n");
	if (stat == 0)
	{
		ROS_INFO("\033[1;32m----> gnss_preprocessor Finished.\033[0m");
	}
	else if (stat > 0)
	{
		ROS_INFO("\033[1;32m----> gnss_preprocessor Error!!!.\033[0m");
	}
	else if (stat == -1)
	{
		ROS_INFO("\033[1;32m----> gnss_preprocessor Aborted!!!.\033[0m");
	}

	return 0;
}

/*****dummy application functions for shared library*****/
extern int showmsg(const char *format,...) {
	va_list arg;
	char buff[1024];
	if (*format) {
		va_start(arg,format);
		vsprintf(buff,format,arg);
		va_end(arg);
		printf("%s\n",buff);
	}
	return 0;	
}
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}
