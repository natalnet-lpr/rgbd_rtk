/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2021, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Authors:
 *
 *  Bruno Silva
 *  Rodrigo Xavier
 */

#include <event_logger.h>
#include <rgbd_loader.h>

#include <pcl/console/print.h>

using namespace std;

int main()
{
    EventLogger& logger = EventLogger::getInstance();
    logger.setVerbosityLevel(EventLogger::L_DEBUG);
    logger.setLogFileName("log_event_logger_test.txt");

    printf("first\n");
    logger.printActiveModules();
    printf("active for tracking? %i\n", logger.isLoggingActiveFor(EventLogger::M_TRACKING));
    logger.deactivateLoggingFor(EventLogger::M_IO);
    logger.deactivateLoggingFor(EventLogger::M_IO);
    logger.deactivateLoggingFor(EventLogger::M_VISUALIZATION);
    printf("second\n");
    logger.printActiveModules();
    printf("active for tracking? %i\n", logger.isLoggingActiveFor(EventLogger::M_TRACKING));
    logger.activateLoggingOnlyFor(EventLogger::M_SLAM);
    printf("third\n");
    logger.printActiveModules();
    printf("active for tracking? %i\n", logger.isLoggingActiveFor(EventLogger::M_TRACKING));
    logger.activateLoggingFor(EventLogger::M_IO);
    logger.activateLoggingFor(EventLogger::M_TRACKING);
    printf("fourth\n");
    logger.printActiveModules();
    printf("active for tracking? %i\n", logger.isLoggingActiveFor(EventLogger::M_TRACKING));

    // These messages are shown on stdout/logged to file according to verbosity level
    logger.print(EventLogger::L_DEBUG, "@event_logger_test.cpp: calling print from main\n");
    logger.print(
        EventLogger::L_INFO, "@event_logger_test.cpp: calling print from main >>> %i, %s, %f\n", 1, "test string", 5.0);
    logger.print(EventLogger::L_WARN, "@event_logger_test.cpp: calling print from main: %s\n", "another string");
    logger.print(EventLogger::L_ERROR, "@event_logger_test.cpp: calling print from main: %f\n", 3.14);

    // Alternative style (does not support formatting)
    logger.printDebug(EventLogger::M_IO, "Testing DEBUG\n");  // goes to file if verb. level >= DEBUG
    logger.printInfo(EventLogger::M_IO, "Testing INFO\n");    // goes to file if verb. level >= INFO
    logger.printWarning(EventLogger::M_IO, "Testing WARN\n"); // goes to file if verb. level >= WARN
    logger.printError(EventLogger::M_IO, "Testing ERROR\n");  // goes to file if verb. level >= ERROR

    // These messages will be printed because they are not debug (even with not active modules)
    logger.printInfo(EventLogger::M_VISUALIZATION, "Will be printed (info)\n");
    logger.printWarning(EventLogger::M_VISUAL_ODOMETRY, "Will be printed (warn)\n");
    logger.printError(EventLogger::M_STEREO, "Will be printed (error)\n");

    // The message below should not be printed because the modules are not active
    logger.printDebug(EventLogger::M_COMMON, "Should not be printed\n");

    // New style (support module and formatting)
    logger.print(EventLogger::L_ERROR, EventLogger::M_IO, "new style call: (%s, %f)\n", "test", 1.0);

    // Using macros
    LOG_ERROR("@event_logger_test.cpp: %f, %s, %i\n", 2.8, "yes", 50);
    MLOG_ERROR(EventLogger::M_IO, "@event_logger_test.cpp: message for an activated module: %f %f\n", 0.5, 4.5);
    // The others are LOG_DEBUG/MLOG_DEBUG, LOG_INFO/MLOG_INFO and LOG_WARN/MLOG_WARN

    // Instantiate a loader to test logging
    RGBDLoader loader;
    loader.processFile("DOES NOT EXIST"); // this will cause an error

    return 0;
}