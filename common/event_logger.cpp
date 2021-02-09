/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2020, Natalnet Laboratory for Perceptual Robotics
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
 *  Authors:
 *
 *  Bruno Silva
 *  Rodrigo Xavier
 */

#include <iostream>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <sstream>

#include <event_logger.h>

using namespace std;

EventLogger& logger = EventLogger::getInstance();

void EventLogger::initialize()
{
	log_file_ = fopen(file_name_.c_str(), "w");
	if(log_file_ == NULL)
	{
		printError("common::EventLogger", "Could not open the specified log file.");
		exit(0);
	}
	is_initialized_ = true;
	print(EventLogger::L_INFO, "[common::EventLogger] INFO: opening file %s\n", EventLogger::file_name_.c_str());
}

EventLogger& EventLogger::getInstance()
{
	static EventLogger logger;
	return logger;
}

void EventLogger::setVerbosityLevel(EventLogger::VERBOSITY_LEVEL level)
{
	verb_level_ = level;
}

void EventLogger::setLogFileName(const string& file_name)
{
	//Close the previously openned file if already initialized
	if(is_initialized_)
		fclose(log_file_);

	//Set the file to be openned with the given param
	//(note that the file is actually openned when some print method is called)
	file_name_ = file_name;
}

EventLogger::VERBOSITY_LEVEL EventLogger::getVerbosityLevel()
{
	return verb_level_;
}

string EventLogger::getLogFileName()
{
	return file_name_;
}

bool EventLogger::isVerbosityLevelEnabled(EventLogger::VERBOSITY_LEVEL level)
{
	return level <= verb_level_;
}

void EventLogger::changeTextColor(FILE *stream, int attribute, int fg)
{
	char command[17];
	sprintf(command, "%c[%d;%dm", 0x1B, attribute, fg + 30);
	fprintf(stream, "%s", command);
}

void EventLogger::resetTextColor(FILE *stream)
{
	char command[13];
  	sprintf(command, "%c[0;m", 0x1B);
	fprintf(stream, "%s", command);
}

/*
 * All functions below are copy/paste from the PCL counterparts
 * because variadic functions don't allow redirecting calls to another
 * variadic function.
 */

void EventLogger::print(EventLogger::VERBOSITY_LEVEL level, const char *format, ...)
{
	if(!isVerbosityLevelEnabled(level))
		return;

	//Change color according to verb. level
	switch(level)
	{
		case EventLogger::L_ERROR:
			changeTextColor(stdout, EventLogger::TT_BRIGHT, EventLogger::TT_RED);
			break;
		case EventLogger::L_WARN:
			changeTextColor(stdout, EventLogger::TT_BRIGHT, EventLogger::TT_YELLOW);
			break;
		case EventLogger::L_DEBUG:
			changeTextColor(stdout, EventLogger::TT_RESET, EventLogger::TT_GREEN);
			break;
		case EventLogger::L_INFO:
		default:
			resetTextColor(stdout);
	}

	va_list stdout_args, file_args;

	//Write message to stdout
	va_start(stdout_args, format);
	va_copy(file_args, stdout_args);
	fprintf(stdout, "[%s] ", currentDateTime().c_str());
	vfprintf(stdout, format, stdout_args);
	va_end(stdout_args);

	//Initialize logger if it was not initialized yet
	if(!is_initialized_)
		initialize();

	//Write message to file
	fprintf(log_file_, "[%s] ", currentDateTime().c_str());
	vfprintf(log_file_, format, file_args);
	va_end(file_args);

	resetTextColor(stdout);
}

void EventLogger::printDebug(const char* module_class, const char* msg)
{
	if(!isVerbosityLevelEnabled(EventLogger::L_DEBUG))
		return;

	if(!is_initialized_)
		initialize();
	print(EventLogger::L_DEBUG, "[%s] DEBUG: %s\n", module_class, msg);
}

void EventLogger::printInfo(const char* module_class, const char* msg)
{
	if(!isVerbosityLevelEnabled(EventLogger::L_INFO))
		return;

	if(!is_initialized_)
		initialize();

	print(EventLogger::L_INFO, "[%s] INFO: %s\n", module_class, msg);
}

void EventLogger::printWarning(const char* module_class, const char* msg)
{
	if(!isVerbosityLevelEnabled(EventLogger::L_WARN))
		return;

	if(!is_initialized_)
		initialize();

	print(EventLogger::L_WARN, "[%s] WARNING: %s\n", module_class, msg);
}

void EventLogger::printError(const char* module_class, const char* msg)
{
	if (!isVerbosityLevelEnabled(EventLogger::L_ERROR))
		return;

	if(!is_initialized_)
		initialize();

	print(EventLogger::L_ERROR, "[%s] ERROR: %s\n", module_class, msg);
}

string EventLogger::currentDateTime()
{
	time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);

    return buf;
}

void EventLogger::activateLoggingOnlyFor(EventLogger::MODULE module)
{
	set<EventLogger::MODULE>::const_iterator it;
    for(it = active_modules_.begin(); it != active_modules_.end(); it++)
    {
        if(*it != module)
        {
        	active_modules_.erase(it);
        }
    }
}