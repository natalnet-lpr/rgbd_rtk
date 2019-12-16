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

#include <event_logger.h>

#include <pcl/console/print.h>

using namespace std;

int main(int argc, char **argv)
{
	EventLogger logger("rgbd_rtk_log.txt");

	//All these messages are shown on screen and logged to file
	logger.print("[common::EventLogger] Info: 1. function call from main\n");
	logger.print("[common::EventLogger] Info: 2. function call from main >>> %i, %s, %f\n", 1, "test string", 5.0);
    logger.print("[common::EventLogger] Info: 3. final function call from main: %s\n", "EXITING");

    //These messages are shown on screen/logged to file according to verbosity level
    pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
	logger.printDebug("common::EventLogger", "Testing DEBUG"); //goes to file if verb. level >= DEBUG
	logger.printInfo("common::EventLogger", "Testing INFO"); //goes to file if verb. level >= INFO
	logger.printWarning("common::EventLogger", "Testing WARN"); //goes to file if verb. level >= WARN
	logger.printError("common::EventLogger", "Testing ERROR"); //goes to file if verb. level >= ERROR

	return 0;
}