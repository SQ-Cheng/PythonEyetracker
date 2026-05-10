from ctypes import *
from enum import Enum

#  Notes: The following structure and enumeration correspond one-to-one to the definitions in the C header file
#  Reference: _7i_types.h


#  2-d coordinate
class py_7i_point2d_t(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float)
    ]


#  3-d coordinate
class py_7i_point3_t(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float)
    ]


#  Eyes type
class PY_7I_EYE_TYPE(Enum):
    L_EYE = 1  # left eye
    R_EYE = 2  # right eye


#  Resolution of the front camera
class PY_7I_RESOLUTION(Enum):
    P1280_960 = 201   # 1280 * 960
    P1280_720 = 202   # 1280 * 720
    P800_600 = 203    # 800  * 600
    P1920_1080 = 204  # 1920 * 1080


#  Type of environment to use
class PY_7I_ENVIRONMENT(Enum):
    INDOOR = 301    # for indoor use
    OUTDOOR = 302   # for outdoor use
    DARKNESS = 303  # use in dark environments


#  calibration coefficient
class py_7i_coefficient_t(Structure):
    _fields_ = [
        ("buf", c_ubyte * 1024)
    ]


#   gaze points
class py_7i_gaze_point_t(Structure):
    _fields_ = [
        ("gaze_bit_mask", c_uint),
        ("gaze_point", py_7i_point3_t),
        ("raw_point", py_7i_point3_t),
        ("smooth_point", py_7i_point3_t),
        ("gaze_origin", py_7i_point3_t),
        ("gaze_direction", py_7i_point3_t),
        ("re", c_float),
        ("ex_data_bit_mask", c_uint),
        ("ex_data", c_float * 32)
    ]


#  pupil info
class py_7i_pupil_info_t(Structure):
    _fields_ = [
        ("pupil_bit_mask", c_uint),
        ("pupil_center", py_7i_point2d_t),
        ("pupil_distance", c_float),
        ("pupil_diameter", c_float),
        ("pupil_diameter_mm", c_float),
        ("pupil_minor_axis", c_float),
        ("pupil_minor_axis_mm", c_float),
        ("ex_data_bit_mask", c_uint),
        ("ex_data", c_float * 32)
    ]


#  extend info
class py_7i_eye_ex_data_t(Structure):
    _fields_ = [
        ("eye_data_ex_bit_mask", c_uint),
        ("blink", c_int),
        ("openness", c_float),
        ("eyelid_up", c_float),
        ("eyelid_down", c_float),
        ("ex_data_bit_mask", c_uint),
        ("ex_data", c_float * 32)
    ]


#  fixation point state
class PY_7I_GAZE_STATE(Structure):
    NONE = 0
    MOVING = 1
    MOVE_END = 2
    STAYING = 3
    STAY_END = 4


#  fixation or saccade
class py_7i_fixation_saccade_t(Structure):
    _fields_ = [
        ("timestamp", c_longlong),
        ("duration", c_longlong),
        ("count", c_int),
        ("state", c_int),
        ("center", py_7i_point2d_t)
    ]


#  gyroscope
class py_7i_gyro_data_t(Structure):
    _fields_ = [
        ("timestamp", c_longlong),
        ("quat_data", c_float * 4),
        ("gyro", c_float * 3),
        ("accel", c_float * 3),
        ("mag", c_float * 3),
        ("extra_data", c_float * 8)
    ]


#  eye data
class py_7i_eye_data_ex_t(Structure):
    _fields_ = [
        ("timestamp", c_ulonglong),
        ("recommend", c_int),
        ("recom_gaze", py_7i_gaze_point_t),
        ("left_gaze", py_7i_gaze_point_t),
        ("right_gaze", py_7i_gaze_point_t),
        ("left_pupil", py_7i_pupil_info_t),
        ("right_pupil", py_7i_pupil_info_t),
        ("left_ex_data", py_7i_eye_ex_data_t),
        ("right_ex_data", py_7i_eye_ex_data_t),
        ("stats", py_7i_fixation_saccade_t),
        ("gyro", py_7i_gyro_data_t)
    ]


#  date and time
class py_7i_date_time_t(Structure):
    _fields_ = [
        ("year", c_int16),
        ("month", c_uint8),
        ("day", c_int8),
        ("hour", c_int8),
        ("minute", c_int8),
        ("second", c_int8)
    ]


#  encrypt device information
class py_7i_ukey_info_t(Structure):
    _fields_ = [
        ("device_id", c_ubyte * 32),
        ("app_id", c_ubyte * 20),
        ("version", c_ubyte * 20),
        ("tip_id", c_ubyte * 20),
        ("end_time", py_7i_date_time_t)
    ]

# image buffer
class py_7i_bytes(Structure):
    _fields_ = [
        ("data", c_ubyte * 6220800)
    ]


#  callback function type
func_camera_state_callback_t = WINFUNCTYPE(None, c_int, py_object)

func_image_callback_t = WINFUNCTYPE(None, c_int, c_void_p, c_int, c_int, c_int, c_longlong, py_object)

func_gaze_callback_t = WINFUNCTYPE(None, POINTER(py_7i_eye_data_ex_t), py_object)

func_point_process_callback_t = WINFUNCTYPE(None, c_int, c_int, py_object)

func_point_finish_callback_t = WINFUNCTYPE(None, c_int, c_int, py_object)


class PY_7I_EYE_GAZE_VALIDITY(Enum):
    ID_EYE_GAZE_POINT = 0
    ID_EYE_GAZE_RAW_POINT = 1
    ID_EYE_GAZE_SMOOTH_POINT = 2
    ID_EYE_GAZE_ORIGIN = 3
    ID_EYE_GAZE_DIRECTION = 4
    ID_EYE_GAZE_RE = 5


class PY_7I_PUPIL_INFO_VALIDITY(Enum):
    ID_PUPIL_INFO_CENTER = 0,
    ID_PUPIL_INFO_DISTANCE = 1,
    ID_PUPIL_INFO_DIAMETER = 2,
    ID_PUPIL_INFO_DIAMETER_MM = 3,
    ID_PUPIL_INFO_MINORAXIS = 4,
    ID_PUPIL_INFO_MINORAXIS_MM = 5


class PY_7I_EYE_EX_DATA_VALIDITY(Enum):
    ID_EYE_EXDATA_BLINK = 0
    ID_EYE_EXDATA_OPENNESS = 1
    ID_EYE_EXDATA_EYELIDUP = 2
    ID_EYE_EXDATA_EYELIDDOWN = 3


class PY_7I_EYE_EX_DATA_EX_DATA_VALIDITY(Enum):
    ID_EYE_EXDATA_PUPIL_LOCATION_X = 0
    ID_EYE_EXDATA_PUPIL_LOCATION_Y = 1
    ID_EYE_EXDATA_PUPIL_LOCATION_Z = 2


class PY_7I_EYE_GAZE_EX_DATA_VALIDITY(Enum):
    ID_EYE_GAZE_EXDATA_SCORE = 0



