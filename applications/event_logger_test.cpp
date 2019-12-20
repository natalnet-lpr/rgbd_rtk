/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2019, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided
 *  that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions and
 *     the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 *     the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rgbd_loader.h>
#include <event_logger.h>

#include <pcl/console/print.h>

using namespace std;

int main(int argc, char **argv)
{
	EventLogger& logger = EventLogger::getInstance();
	logger.setVerbosityLevel(pcl::console::L_DEBUG);
	logger.setLogFileName("log_event_logger_test.txt");

	//These messages are shown on stdout/logged to file according to verbosity level
	logger.print(pcl::console::L_DEBUG, "[event_logger_test.cpp] DEBUG: calling print from main\n");
	logger.print(pcl::console::L_INFO, "[event_logger_test.cpp] INFO: calling print from main >>> %i, %s, %f\n", 1, "test string", 5.0);
    logger.print(pcl::console::L_WARN, "[event_logger_test.cpp] WARN: calling print from main: %s\n", "another string");
    logger.print(pcl::console::L_ERROR, "[event_logger_test.cpp] ERROR: calling print from main: %f\n", 3.14);

	logger.printDebug("event_logger_test.cpp", "Testing DEBUG"); //goes to file if verb. level >= DEBUG
	logger.printInfo("event_logger_test.cpp", "Testing INFO"); //goes to file if verb. level >= INFO
	logger.printWarning("event_logger_test.cpp", "Testing WARN"); //goes to file if verb. level >= WARN
	logger.printError("event_logger_test.cpp", "Testing ERROR"); //goes to file if verb. level >= ERROR

	//Instantiate a loader to test logging
	RGBDLoader loader;
	loader.processFile("DOESNOT EXIST"); //this will cause an error

	return 0;
}