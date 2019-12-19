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

#ifndef INCLUDE_EVENT_LOGGER_H_
#define INCLUDE_EVENT_LOGGER_H_

#include <cstdio>
#include <iostream>

#include <pcl/console/print.h>

class EventLogger
{

private:

	//Informs if the logger has been initialized
	//(whether it has correctly openned a file)
	bool is_initialized_;

	//Name of the log file
	std::string file_name_;

	//Output file for the log file
	FILE* log_file_;

	//Current verbosity level (uses enum defined in PCL)
	pcl::console::VERBOSITY_LEVEL verb_level_;

	//Associates the logger with a FILE*
	void initialize();

	/**
	 * Default (private) constructor:
	 * the default log file name is log.txt
	 * (which will be created in the binary dir.)
	 * and the verbosity level is L_ERROR.
	 * Note that the file is only actually openned
	 * when one of the print member functions are called.
	 */
	EventLogger()
	{
		file_name_ = "log.txt";
		is_initialized_ = false;
		verb_level_ = pcl::console::L_ERROR;
	}

	//EventLogger(EventLogger const&); //don't implement copy constructor
	//void operator=(EventLogger const&); //don't implement assignment operator

public:

	/** 
	 * Class destructor.
	 */
	~EventLogger()
	{
		if(is_initialized_)
			fclose(log_file_);
	}

	/**
	 * Returns the single instance of the class
	 * currently in execution.
	 */
	static EventLogger& getInstance();

	/**
	 * Sets the verbosity level.
	 * @param level - the following are accepted: L_DEBUG, L_INFO, L_WARN, L_ERROR
	 */
    void setVerbosityLevel(pcl::console::VERBOSITY_LEVEL level);

    /**
     * Sets the log file name.
     * @param file_name: log file name (supports relative/absolute path)
     */
    void setLogFileName(const std::string& file_name);

    /**
     * Returns the verbosity level.
     */
    pcl::console::VERBOSITY_LEVEL getVerbosityLevel();

    /**
     * Returns the log file name.
     */
    std::string getLogFileName();

    /**
     * Returns true if the given verbosity level is enabled.
     * @param level - the following are accepted: L_DEBUG, L_INFO, L_WARN, L_ERROR
     */
    bool isVerbosityLevelEnabled(pcl::console::VERBOSITY_LEVEL level);

	void print(const char* format, ...);

	void printDebug(const char* module_class, const char* msg);

	void printInfo(const char* module_class, const char* msg);

	void printWarning(const char* module_class, const char* msg);

	void printError(const char* module_class, const char* msg);
};

#endif /* INCLUDE_EVENT_LOGGER_H_ */