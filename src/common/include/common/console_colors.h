#ifndef CONSOLE_COLORS
#define CONSOLE_COLORS

/*!
 * Some color definitions for the command line.
 * Reference: http://www.cplusplus.com/forum/unices/36461/
 *
 * example usage:
 *
 * ROS_DEBUG_STREAM("normal debug text" << COLOR_RED <<
 *                  "red debug text" << COLOR_DEBUG <<
 *                  "normal debug text again");
 *
 */

#define COLOR_NORMAL "\033[0m"
#define COLOR_BLACK "\033[30m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_BLUE "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN "\033[36m"
#define COLOR_LIGHT_GREY "\033[37m"

// Colors in terms of ROS logger levels

#define COLOR_DEBUG COLOR_GREEN
#define COLOR_INFO COLOR_NORMAL
#define COLOR_WARN COLOR_YELLOW
#define COLOR_ERROR COLOR_RED

#endif // CONSOLE_COLORS

