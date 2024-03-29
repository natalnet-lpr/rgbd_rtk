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
 *
 * (Various chunks of codes were taken from Point Clouds Library).
 */

#ifndef INCLUDE_EVENT_LOGGER_H_
#define INCLUDE_EVENT_LOGGER_H_

#include <cstdio>
#include <map>
#include <set>
#include <string>

#define LOG_ERROR(...) logger.print(EventLogger::L_ERROR, __VA_ARGS__)
#define LOG_WARN(...) logger.print(EventLogger::L_WARN, __VA_ARGS__)
#define LOG_INFO(...) logger.print(EventLogger::L_INFO, __VA_ARGS__)
#define LOG_DEBUG(...) logger.print(EventLogger::L_DEBUG, __VA_ARGS__)

#define MLOG_ERROR(MOD, ...) logger.print(EventLogger::L_ERROR, MOD, __VA_ARGS__)
#define MLOG_WARN(MOD, ...) logger.print(EventLogger::L_WARN, MOD, __VA_ARGS__)
#define MLOG_INFO(MOD, ...) logger.print(EventLogger::L_INFO, MOD, __VA_ARGS__)
#define MLOG_DEBUG(MOD, ...) logger.print(EventLogger::L_DEBUG, MOD, __VA_ARGS__)

class EventLogger
{
public:
    enum VERBOSITY_LEVEL
    {
        L_ALWAYS,
        L_ERROR,
        L_WARN,
        L_INFO,
        L_DEBUG,
        L_VERBOSE
    };

    enum MODULE
    {
        M_IO = 0,
        M_COMMON = 1,
        M_TRACKING = 2,
        M_MOTION_ESTIMATION = 3,
        M_STEREO = 4,
        M_VISUALIZATION = 5,
        M_VISUAL_ODOMETRY = 6,
        M_SLAM = 7,
        M_SEGMENTATION = 8
    };

    /**
     * Class destructor.
     */
    ~EventLogger()
    {
        if (is_initialized_) fclose(log_file_);
    }

    /**
     * @return the single instance of the class
     * currently in execution.
     */
    static EventLogger& getInstance();

    /**
     * Sets the verbosity level.
     * @param level the following are accepted: L_DEBUG, L_INFO, L_WARN, L_ERROR
     */
    void setVerbosityLevel(EventLogger::VERBOSITY_LEVEL level);

    /**
     * Sets the log file name.
     * @param file_name: log file name (supports relative/absolute path)
     */
    void setLogFileName(const std::string& file_name);

    /**
     * @return verbosity level.
     */
    EventLogger::VERBOSITY_LEVEL getVerbosityLevel();

    /**
     * @return the log file name.
     */
    std::string getLogFileName();

    /**
     * Change the text color (on stdout, stderr, etc.) with an attr:fg:bg
     * @param stream the output stream (stdout, stderr, etc)
     * @param attribute the text attribute
     * @param fg the foreground color
     */
    void changeTextColor(FILE* stream, int attribute, int fg);

    /**
     * Reset the text color (on either stdout, stderr, etc) to its original state
     * @param stream the output stream
     */
    void resetTextColor(FILE* stream);

    /**
     * @param level - the following are accepted: L_DEBUG, L_INFO, L_WARN, L_ERROR
     * @return true if the given verbosity level is enabled.
     */
    bool isVerbosityLevelEnabled(EventLogger::VERBOSITY_LEVEL level);

    /**
     * @param module: M_IO, M_COMMON, M_TRACKING, etc.
     * @return true if logging is active for the given module.
     */
    bool isLoggingActiveFor(EventLogger::MODULE module);

    /**
     * Prints a formatted message to stdout/file with informed
     * verbosity level.
     * It works just like printf.
     * Example use:
     *
     * print(EventLogger::L_DEBUG, "Formatted message: %i, %f\n", var1, var2);
     * @param level: verbosity levels (L_DEBUG, L_INFO, L_WARN or L_ERROR).
     * @param format: formatted string and its list of variables
     */
    void print(EventLogger::VERBOSITY_LEVEL level, const char* format, ...);

    /**
     * Prints a formatted message to stdout/file with informed
     * verbosity level and library module.
     * It works just like printf.
     * Example use:
     *
     * print(EventLogger::L_DEBUG, EventLogger::M_IO, "Formatted message: %i, %f\n", var1, var2);
     * @param level: verbosity levels (L_DEBUG, L_INFO, L_WARN or L_ERROR).
     * @param module: M_IO, M_COMMON, M_TRACKING, etc.
     * @param format: formatted string and its list of variables
     */
    void
    print(EventLogger::VERBOSITY_LEVEL level, EventLogger::MODULE module, const char* format, ...);

    /**
     * Prints a message to stdout/file to denote debug information.
     * It redirects the call to the print function for proper formatting
     * (note that this function receives only two parameters).
     * Example use:
     *
     * printDebug(EventLogger::M_TRACKING, "Debug message for the tracking module\n");
     * @param module: M_IO, M_COMMON, M_TRACKING, etc.
     * @param msg: string with a message
     */
    void printDebug(EventLogger::MODULE module, const char* msg);

    /**
     * Prints a message to stdout/file to denote general program information.
     * It redirects the call to the print function for proper formatting
     * (note that this function receives only two parameters).
     * Example use:
     *
     * printInfo(EventLogger::M_TRACKING, "Information message for the tracking module\n");
     * @param module: M_IO, M_COMMON, M_TRACKING, etc.
     * @param msg: string with a message
     */
    void printInfo(EventLogger::MODULE module, const char* msg);

    /**
     * Prints a message to stdout/file to denote a program warning.
     * It redirects the call to the print function for proper formatting
     * (note that this function receives only two parameters).
     * Example use:
     *
     * printWarning(EventLogger::M_TRACKING, "Warning message for the tracking module\n");
     * @param module: M_IO, M_COMMON, M_TRACKING, etc.
     * @param msg: string with a message
     */
    void printWarning(EventLogger::MODULE module, const char* msg);

    /**
     * Prints a message to stdout/file to denote an error situation.
     * It redirects the call to the print function for proper formatting
     * (note that this function receives only two parameters).
     * Example use:
     *
     * printError(EventLogger::M_TRACKING, "Error message for the tracking module\n");
     * @param module: M_IO, M_COMMON, M_TRACKING, etc.
     * @param msg: string with a message
     */
    void printError(EventLogger::MODULE module, const char* msg);

    /**
     * @return a string with the current local data and time, Format YYYY-MM-DD HH:mm:ss
     */
    std::string currentDateTime();

    /**
     * Activates logging for the specified module.
     * Does nothing if already activated for the specified module.
     * @param module desire: M_IO, M_TRACKING, etc.
     */
    void activateLoggingFor(EventLogger::MODULE module);

    /**
     * Deactivates logging for the specified module.
     * Does nothing if logging was already excluded for the specified module.
     * @param module desire: M_IO, M_TRACKING, etc.
     */
    void deactivateLoggingFor(EventLogger::MODULE module);

    /**
     * Activates logging only for the specified module,
     * excluding all others.
     * @param desired module: M_IO, M_TRACKING, etc.
     */
    void activateLoggingOnlyFor(EventLogger::MODULE module);

    // DEBUG ONLY
    void printActiveModules();

private:
    enum TT_ATTRIBUTES
    {
        TT_RESET = 0,
        TT_BRIGHT = 1,
        TT_DIM = 2,
        TT_UNDERLINE = 3,
        TT_BLINK = 4,
        TT_REVERSE = 7,
        TT_HIDDEN = 8
    };

    enum TT_COLORS
    {
        TT_BLACK,
        TT_RED,
        TT_GREEN,
        TT_YELLOW,
        TT_BLUE,
        TT_MAGENTA,
        TT_CYAN,
        TT_WHITE
    };

    // Informs if the logger has been initialized
    // (whether it has correctly openned a file)
    bool is_initialized_;

    // Name of the log file
    std::string file_name_;

    // Output file for the log file
    FILE* log_file_;

    // Current verbosity level
    EventLogger::VERBOSITY_LEVEL verb_level_;

    // Associates the logger with a FILE*
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
        verb_level_ = EventLogger::L_ERROR;

        // All modules are active by default
        active_modules_.insert(M_IO);
        active_modules_.insert(M_COMMON);
        active_modules_.insert(M_TRACKING);
        active_modules_.insert(M_MOTION_ESTIMATION);
        active_modules_.insert(M_STEREO);
        active_modules_.insert(M_VISUALIZATION);
        active_modules_.insert(M_VISUAL_ODOMETRY);
        active_modules_.insert(M_SLAM);
        active_modules_.insert(M_SEGMENTATION);

        // Initialize map with the name of each module
        modules_names_[M_IO] = "IO";
        modules_names_[M_COMMON] = "Common";
        modules_names_[M_TRACKING] = "Tracking";
        modules_names_[M_MOTION_ESTIMATION] = "MotionEstimation";
        modules_names_[M_STEREO] = "Stereo";
        modules_names_[M_VISUALIZATION] = "Visualization";
        modules_names_[M_VISUAL_ODOMETRY] = "VisualOdometry";
        modules_names_[M_SLAM] = "SLAM";
        modules_names_[M_SEGMENTATION] = "SEGMENTATION";
    }

    // Set of active modules
    std::set<EventLogger::MODULE> active_modules_;

    // Maps a module constant into its descriptive string
    std::map<EventLogger::MODULE, std::string> modules_names_;

    EventLogger(EventLogger const&);    // don't implement copy constructor
    void operator=(EventLogger const&); // don't implement assignment operator
};

extern EventLogger& logger;

#endif /* INCLUDE_EVENT_LOGGER_H_ */