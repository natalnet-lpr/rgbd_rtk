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

#include <event_logger.h>

#include <pcl/console/print.h>

using namespace pcl::console;

/*
 * All functions below are copy/paste from the PCL counterparts
 * because variadic functions don't allow redirecting calls to another
 * variadic function.
 */

void EventLogger::print(const char *format, ...)
{
	va_list stdout_args, file_args;

	//Write message to stdout and file
	va_start(stdout_args, format);
	va_copy(file_args, stdout_args);
	vfprintf(stdout, format, stdout_args);
	vfprintf(log_file_, format, file_args);
	va_end(stdout_args);
	va_end(file_args);
}

void EventLogger::printDebug(const char* module_class, const char* msg)
{
	if(!isVerbosityLevelEnabled(L_DEBUG))
		return;

	change_text_color (stdout, TT_RESET, TT_GREEN);

	print("[%s] DEBUG: %s\n", module_class, msg);
}

void EventLogger::printInfo(const char* module_class, const char* msg)
{
	if(!isVerbosityLevelEnabled(L_INFO))
		return;

	reset_text_color(stdout);

	print("[%s] INFO: %s\n", module_class, msg);
}

void EventLogger::printWarning(const char* module_class, const char* msg)
{
	if(!isVerbosityLevelEnabled(L_WARN))
		return;

	change_text_color(stderr, TT_BRIGHT, TT_YELLOW);

	print("[%s] WARNING: %s\n", module_class, msg);
}

void EventLogger::printError(const char* module_class, const char* msg)
{
	if(!isVerbosityLevelEnabled(L_ERROR))
		return;

	change_text_color(stderr, TT_BRIGHT, TT_RED);

	print("[%s] ERROR: %s\n", module_class, msg);
}