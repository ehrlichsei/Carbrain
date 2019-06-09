#ifndef BIRDS_VIEW_TRANSFORMATION_PARAMETERS
#define BIRDS_VIEW_TRANSFORMATION_PARAMETERS

#include "common/parameter_interface.h"

namespace common {
/*!
 * \brief CAMERA_FOCAL_LENGTH_X
 *
 * Focal length in x-direction. Unit: mm??
 */
const ParameterString<double> CAMERA_FOCAL_LENGTH_X("/camera/focal_length_x");
/*!
 * \brief CAMERA_FOCAL_LENGTH_Y
 *
 * Focal length in y-direction. Unit: mm??
 */
const ParameterString<double> CAMERA_FOCAL_LENGTH_Y("/camera/focal_length_y");
/*!
 * \brief CAMERA_OPTICAL_CENTER_X
 *
 * Optical center in x-direction. Unit: pixel??
 */
const ParameterString<double> CAMERA_OPTICAL_CENTER_X("/camera/optical_center_x");
/*!
 * \brief CAMERA_OPTICAL_CENTER_Y
 *
 * Optical center in y-direction. Unit: pixel??
 */
const ParameterString<double> CAMERA_OPTICAL_CENTER_Y("/camera/optical_center_y");

const ParameterString<double> CAMERA_R11("/camera/r11");
const ParameterString<double> CAMERA_R12("/camera/r12");
const ParameterString<double> CAMERA_R13("/camera/r13");
const ParameterString<double> CAMERA_R21("/camera/r21");
const ParameterString<double> CAMERA_R22("/camera/r22");
const ParameterString<double> CAMERA_R23("/camera/r23");
const ParameterString<double> CAMERA_R31("/camera/r31");
const ParameterString<double> CAMERA_R32("/camera/r32");
const ParameterString<double> CAMERA_R33("/camera/r33");

const ParameterString<double> CAMERA_T1("/camera/t1");
const ParameterString<double> CAMERA_T2("/camera/t2");
const ParameterString<double> CAMERA_T3("/camera/t3");

} // namespace common

#endif // BIRDS_VIEW_TRANSFORMATION_PARAMETERS
