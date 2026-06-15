"""VR SDK ctypes 类型定义 — 对应 aSeeVRClient.h / aSeeVRTypes.h / aSeeVRUtility.h"""

from ctypes import (
    Structure, Union, c_int, c_int16, c_int32, c_int64,
    c_uint8, c_uint32, c_float, c_void_p, WINFUNCTYPE, py_object,
)


# ---- Enums ----
class ASeeVRCallbackType:
    state      = 0
    eye_data   = 1
    eye_image  = 2
    coefficient = 3


class ASeeVRStateCode:
    api_start               = 1001
    api_stop                = 1002
    api_get_coefficient     = 1009


class ASeeVRReturnCode:
    bind_local_port_failed  = -5
    permission_denied       = -4
    invalid_value           = -3
    invalid_parameter       = -2
    failed                  = -1
    success                 = 0


class ASeeVREye:
    undefine_eye = -1
    left_eye     = 0
    right_eye    = 1


class ASeeVRClientMode:
    control = 1
    monitor = 2


# ---- Data item types for aSeeVREyeData extraction ----
class ASeeVREyeDataItemType:
    timestamp           = 0
    recommend           = 1
    gaze                = 2    # aSeeVRPoint2D
    gaze_raw            = 3    # aSeeVRPoint2D
    gaze_smooth         = 4    # aSeeVRPoint2D
    gaze_origin         = 5    # aSeeVRPoint3D
    gaze_direction      = 6    # aSeeVRPoint3D
    gaze_reliability    = 7    # float
    pupil_center        = 8    # aSeeVRPoint2D (归一化 0~1)
    pupil_distance      = 9    # float (mm)
    pupil_diameter      = 10   # float (归一化 0~1)
    pupil_diameter_mm   = 11   # float (mm)
    pupil_minoraxis     = 12   # float (归一化 0~1)
    pupil_minoraxis_mm  = 13   # float (mm)
    blink               = 14   # int32
    openness            = 15   # float
    upper_eyelid        = 16   # float
    lower_eyelid        = 17   # float


# ---- Structures ----
class _point2d_s(Structure):
    _fields_ = [("x", c_float), ("y", c_float)]

class ASeeVRPoint2D(Union):
    _fields_ = [
        ("s",   _point2d_s),
        ("seq", c_float * 2),
    ]
    @property
    def x(self): return self.s.x
    @x.setter
    def x(self, v): self.s.x = v
    @property
    def y(self): return self.s.y
    @y.setter
    def y(self, v): self.s.y = v


class _point3d_s(Structure):
    _fields_ = [("x", c_float), ("y", c_float), ("z", c_float)]

class ASeeVRPoint3D(Union):
    _fields_ = [
        ("s",   _point3d_s),
        ("seq", c_float * 3),
    ]
    @property
    def x(self): return self.s.x
    @x.setter
    def x(self, v): self.s.x = v
    @property
    def y(self): return self.s.y
    @y.setter
    def y(self, v): self.s.y = v
    @property
    def z(self): return self.s.z
    @z.setter
    def z(self, v): self.s.z = v


class ASeeVRImage(Structure):
    _fields_ = [
        ("flag",      c_int32),     # 1=left eye, 2=right eye
        ("width",     c_int32),
        ("height",    c_int32),
        ("data",      c_void_p),    # uint8_t* (grayscale)
        ("timestamp", c_int64),
    ]


class ASeeVRState(Structure):
    _fields_ = [
        ("code",  c_int32),   # ASeeVRStateCode
        ("error", c_int32),   # 0=success
    ]


class ASeeVRInitParam(Structure):
    _fields_ = [
        ("mode",  c_int32),           # ASeeVRClientMode
        ("ports", c_int16 * 10),      # server port numbers
    ]


class ASeeVRLanuchParam(Structure):
    _fields_ = [
        ("enable_iris", c_int32),    # 1=enable, 0=disable
        ("eye",         c_int32),    # 1=left, 2=right, 3=binocular
    ]


class ASeeVRCoefficient(Structure):
    _fields_ = [
        ("buf", c_uint8 * 2048),
    ]


# aSeeVREyeData is opaque — we only hold a pointer, never allocate it ourselves
class ASeeVREyeData(Structure):
    pass


# ---- Callback types ----
StateCallback = WINFUNCTYPE(None, c_void_p,   py_object)  # (const aSeeVRState*, context)
EyeDataCallback = WINFUNCTYPE(None, c_void_p,  py_object)  # (const aSeeVREyeData*, context)
EyeImageCallback = WINFUNCTYPE(None, c_void_p, py_object)  # (const aSeeVRImage*, context)
CoefficientCallback = WINFUNCTYPE(None, c_void_p, py_object)  # (const aSeeVRCoefficient*, context)
