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

#include <iostream>
#include <cstdarg>
#include <cstdio>

#include <event_logger.h>

using namespace std;

void EventLogger::initialize()
{
	log_file_ = fopen(EventLogger::file_name_.c_str(), "w");
	if(log_file_ == NULL)
	{
		printError("common::EventLogger", "Could not open the specified log file.");
		exit(0);
	}
	is_initialized_ = true;
	print(pcl::console::L_INFO, "[common::EventLogger] INFO: opening file %s\n", EventLogger::file_name_.c_str());
}

EventLogger& EventLogger::getInstance()
{
	static EventLogger logger;
	return logger;
}

void EventLogger::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL level)
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
	EventLogger::file_name_ = file_name;
}

pcl::console::VERBOSITY_LEVEL EventLogger::getVerbosityLevel()
{
	return verb_level_;
}

string EventLogger::getLogFileName()
{
	return EventLogger::file_name_;
}

bool EventLogger::isVerbosityLevelEnabled(pcl::console::VERBOSITY_LEVEL level)
{
	return level <= verb_level_;
}

/*
 * All functions below are copy/paste from the PCL counterparts
 * because variadic functions don't allow redirecting calls to another
 * variadic function.
 */

void EventLogger::print(pcl::console::VERBOSITY_LEVEL level, const char *format, ...)
{
	if(!isVerbosityLevelEnabled(level))
		return;

	//Change color according to verb. level
	switch(level)
	{
		case pcl::console::L_ERROR:
			pcl::console::change_text_color(stdout, pcl::console::TT_BRIGHT, pcl::console::TT_RED);
			break;
		case pcl::console::L_WARN:
			pcl::console::change_text_color(stdout, pcl::console::TT_BRIGHT, pcl::console::TT_YELLOW);
			break;
		case pcl::console::L_DEBUG:
			pcl::console::change_text_color(stdout, pcl::console::TT_RESET, pcl::console::TT_GREEN);
			break;
		case pcl::console::L_INFO:
		default:
			pcl::console::reset_text_color(stdout);

	}

	va_list stdout_args, file_args;

	//Write message to stdout
	va_start(stdout_args, format);
	va_copy(file_args, stdout_args);
	vfprintf(stdout, format, stdout_args);
	va_end(stdout_args);

	//Initialize logger if it was not initialized yet
	if(!is_initialized_)
		initialize();

	//Write message to file
	vfprintf(log_file_, format, file_args);
	va_end(file_args);

	pcl::console::reset_text_color(stdout);
}

void EventLogger::printDebug(const char* module_class, const char* msg)
{
	if(!isVerbosityLevelEnabled(pcl::console::L_DEBUG))
		return;

	if(!is_initialized_)
		initialize();

	print(pcl::console::L_DEBUG, "[%s] DEBUG: %s\n", module_class, msg);
}

void EventLogger::printInfo(const char* module_class, const char* msg)
{
	if(!isVerbosityLevelEnabled(pcl::console::L_INFO))
		return;

	if(!is_initialized_)
		initialize();

	print(pcl::console::L_INFO, "[%s] INFO: %s\n", module_class, msg);
}

void EventLogger::printWarning(const char* module_class, const char* msg)
{
	if(!isVerbosityLevelEnabled(pcl::console::L_WARN))
		return;

	if(!is_initialized_)
		initialize();

	print(pcl::console::L_WARN, "[%s] WARNING: %s\n", module_class, msg);
}

void EventLogger::printError(const char* module_class, const char* msg)
{
	if(!isVerbosityLevelEnabled(pcl::console::L_ERROR))
		return;

	if(!is_initialized_)
		initialize();

	print(pcl::console::L_ERROR, "[%s] ERROR: %s\n", module_class, msg);
}