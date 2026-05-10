import os
import queue
import threading
import time
from sdk_types import *
import ctypes

class wrapper:
    ui_handle = None

    h256_dll_handle = None
    sdk_dll_handle = None
    sdk_config_path = None

    py_camera_state_cb = None
    py_image_cb = None
    py_gaze_cb = None
    py_left_point_process_cb = None
    py_right_point_process_cb = None
    py_left_point_finish_cb = None
    py_right_point_finish_cb = None

    left_img_count = 0
    right_img_count = 0

    thread_handle = None
    select_Point_event = threading.Event()
    flag_exit_thread = False
    thread_is_running = False
    calib_points = None
    cur_point_x = 0.0
    cur_point_y = 0.0

    scene_img_size_max = 1920 * 1080 * 3 

    def set_current_point(self, x, y):
        self.cur_point_x = float(x)
        self.cur_point_y = float(y)
        self.select_Point_event.set()
        print('set_current_point %f %f' % (self.cur_point_x, self.cur_point_y))

    @staticmethod
    def set_ui_handle(ui_object):
        wrapper.ui_handle = ui_object

    @staticmethod
    def camera_state_callback(state, context):
        self = context
        print('enter camera_state_callback')
        print('state:%d' % state)

    @staticmethod
    def image_callback(eye, image, size, width, height, timestamp, context):
        self = context
        pc_arrival_timestamp = time.perf_counter()
        ui = wrapper.ui_handle
        if 13 == eye:  # scene images
            if ui and hasattr(ui, "handle_scene_frame"):
                if hasattr(ui, "wants_preview_frame") and not ui.wants_preview_frame(eye):
                    return
                try:
                    frame_bytes = ctypes.string_at(image, int(size))
                    ui.handle_scene_frame(frame_bytes, int(size), int(width), int(height), int(timestamp), pc_arrival_timestamp)
                except Exception:
                    pass
        else:
            if PY_7I_EYE_TYPE.L_EYE.value == eye:
                if ui and hasattr(ui, "handle_eye_preview_frame"):
                    if hasattr(ui, "wants_preview_frame") and not ui.wants_preview_frame(eye):
                        return
                    try:
                        frame_bytes = ctypes.string_at(image, int(width) * int(height))
                        ui.handle_eye_preview_frame(int(eye), frame_bytes, int(width), int(height), int(timestamp), pc_arrival_timestamp)
                    except Exception:
                        pass
            elif PY_7I_EYE_TYPE.R_EYE.value == eye:
                if ui and hasattr(ui, "handle_right_eye_frame"):
                    ui.handle_right_eye_frame(image, width, height, int(timestamp), pc_arrival_timestamp)
                if ui and hasattr(ui, "handle_eye_preview_frame"):
                    if hasattr(ui, "wants_preview_frame") and not ui.wants_preview_frame(eye):
                        return
                    try:
                        frame_bytes = ctypes.string_at(image, int(width) * int(height))
                        ui.handle_eye_preview_frame(int(eye), frame_bytes, int(width), int(height), int(timestamp), pc_arrival_timestamp)
                    except Exception:
                        pass

    @staticmethod
    def gaze_callback(eyes, context):
        self = context
        pc_arrival_timestamp = time.perf_counter()
        obj = eyes.__getitem__(0)

        sample = {
            "pc_arrival_timestamp": pc_arrival_timestamp,
            "device_timestamp": int(obj.timestamp),
            "gaze_x": float(obj.recom_gaze.gaze_point.x),
            "gaze_y": float(obj.recom_gaze.gaze_point.y),
            "gaze_z": float(obj.recom_gaze.gaze_point.z),
            "left_pupil_x": float(obj.left_pupil.pupil_center.x),
            "left_pupil_y": float(obj.left_pupil.pupil_center.y),
            "right_pupil_x": float(obj.right_pupil.pupil_center.x),
            "right_pupil_y": float(obj.right_pupil.pupil_center.y),
            "left_pupil_diameter_mm": float(obj.left_pupil.pupil_diameter_mm),
            "right_pupil_diameter_mm": float(obj.right_pupil.pupil_diameter_mm),
            "left_blink": int(obj.left_ex_data.blink),
            "right_blink": int(obj.right_ex_data.blink),
            "left_openness": float(obj.left_ex_data.openness),
            "right_openness": float(obj.right_ex_data.openness),
        }
        try:
            self.data_queue.put(sample)
        except Exception:
            pass


    @staticmethod
    def left_point_process_callback(index, percent, context):
        self = context
        print("left process:%d %d" % (index, percent))

    @staticmethod
    def right_point_process_callback(index, percent, context):
        self = context
        print("right process:%d %d" % (index, percent))

    @staticmethod
    def left_point_finish_callback(index, error, context):
        self = context
        print("left finish:%d %d" % (index, error))
        if wrapper.ui_handle and hasattr(wrapper.ui_handle, "set_calibration_finish_signal"):
            wrapper.ui_handle.set_calibration_finish_signal.emit(0, index, error)

    @staticmethod
    def right_point_finish_callback(index, error, context):
        self = context
        print("right finish:%d %d" % (index, error))
        if wrapper.ui_handle and hasattr(wrapper.ui_handle, "set_calibration_finish_signal"):
            wrapper.ui_handle.set_calibration_finish_signal.emit(1, index, error)

    def __init__(self):
        self.py_status_cb = func_camera_state_callback_t(wrapper.camera_state_callback)
        self.py_image_cb = func_image_callback_t(wrapper.image_callback)
        self.py_gaze_cb = func_gaze_callback_t(wrapper.gaze_callback)
        self.py_left_point_process_cb = func_point_process_callback_t(wrapper.left_point_process_callback)
        self.py_left_point_finish_cb = func_point_finish_callback_t(wrapper.left_point_finish_callback)
        self.py_right_point_process_cb = func_point_process_callback_t(wrapper.right_point_process_callback)
        self.py_right_point_finish_cb = func_point_finish_callback_t(wrapper.right_point_finish_callback)
        self.data_queue = queue.Queue()
        self.scene_img_buf = py_7i_bytes()


    def load_library(self, path):
        if isinstance(path, bytes):
            config_path = path.decode("utf-8")
            self.sdk_config_path = path  # path to the SDK configuration file
        else:
            config_path = path
            self.sdk_config_path = path.encode("utf-8")

        base_dir = os.path.abspath(os.path.join(config_path, os.pardir))
        self.sdk_dll_handle = ctypes.WinDLL(os.path.join(base_dir, "aSeeX.dll"))
        self.h256_dll_handle = ctypes.WinDLL(os.path.join(base_dir, "H265Decode.dll"))
        print(self.sdk_dll_handle)

    def connect_softdog(self, password) -> int:
        ukey_info = py_7i_ukey_info_t()
        ptr_ukey_info = pointer(ukey_info)
        ret = self.sdk_dll_handle._7i_device_connect(password, ptr_ukey_info)
        print('_7i_device_connect:%d' % ret)
        return ret

    def start(self, environment, resolution, img_width, img_height) -> int:
        self.left_img_count = 0
        self.right_img_count = 0
        self.sdk_dll_handle._7i_set_camera_state_callback(self.py_status_cb, py_object(self))
        self.sdk_dll_handle._7i_set_image_callback(self.py_image_cb, py_object(self))
        self.sdk_dll_handle._7i_set_gaze_callback(self.py_gaze_cb, py_object(self))

        ret = self.h256_dll_handle._7i_h265_init(img_width, img_height)
        print('_7i_h265_init:%d' % ret)

        enable_gyroscope = 0
        print('sdk config path:%s' % self.sdk_config_path)
        ret = self.sdk_dll_handle._7i_start(self.sdk_config_path, environment, resolution, enable_gyroscope)
        print('_7i_start:%d' % ret)

        if ret == 0:
            self._load_base_coefficients()

        return ret

    def _load_base_coefficients(self):
        try:
            if isinstance(self.sdk_config_path, bytes):
                config_dir = self.sdk_config_path.decode("utf-8")
            else:
                config_dir = self.sdk_config_path
            base_coe_path = os.path.join(config_dir, "base_coe.dat")
            if not os.path.isfile(base_coe_path):
                print("[WARN] base_coe.dat not found at %s" % base_coe_path)
                return
            coe = py_7i_coefficient_t()
            with open(base_coe_path, "rb") as f:
                data = f.read(1024)
            memmove(coe.buf, data, len(data))
            left_buf_len = c_int(1024)
            ret1 = self.sdk_dll_handle._7i_set_data(b'biLeft', len(b'biLeft'), coe.buf, left_buf_len)
            right_buf_len = c_int(1024)
            ret2 = self.sdk_dll_handle._7i_set_data(b'biRight', len(b'biRight'), coe.buf, right_buf_len)
            print("_7i_set_data base_coe: L=%d R=%d" % (ret1, ret2))
        except Exception as e:
            print("[WARN] Failed to load base coefficients: %s" % e)

    def _read_active_coefficients(self):
        left_coe = py_7i_coefficient_t()
        right_coe = py_7i_coefficient_t()
        left_buf_len = c_int(1024)
        right_buf_len = c_int(1024)
        ret_left = self.sdk_dll_handle._7i_get_data(b'biLeft', len(b'biLeft'), left_coe.buf, byref(left_buf_len))
        ret_right = self.sdk_dll_handle._7i_get_data(b'biRight', len(b'biRight'), right_coe.buf, byref(right_buf_len))
        if ret_left != 0 or ret_right != 0:
            raise RuntimeError("failed to read active calibration coefficients: L=%d R=%d" % (ret_left, ret_right))
        return bytes(left_coe.buf), bytes(right_coe.buf)

    def save_calibration_profile(self, profile_dir):
        os.makedirs(profile_dir, exist_ok=True)
        left_data, right_data = self._read_active_coefficients()
        with open(os.path.join(profile_dir, 'left_coe.dat'), 'wb') as fp:
            fp.write(left_data)
        with open(os.path.join(profile_dir, 'right_coe.dat'), 'wb') as fp:
            fp.write(right_data)

    def load_calibration_profile(self, profile_dir):
        left_path = os.path.join(profile_dir, 'left_coe.dat')
        right_path = os.path.join(profile_dir, 'right_coe.dat')
        if not os.path.isfile(left_path) or not os.path.isfile(right_path):
            raise FileNotFoundError("calibration profile files are missing")

        local_left_coe = py_7i_coefficient_t()
        with open(left_path, 'rb') as fp:
            left_data = fp.read(1024)
            memmove(local_left_coe.buf, left_data, len(left_data))

        local_right_coe = py_7i_coefficient_t()
        with open(right_path, 'rb') as fp:
            right_data = fp.read(1024)
            memmove(local_right_coe.buf, right_data, len(right_data))

        left_buf_len = c_int(1024)
        right_buf_len = c_int(1024)
        ret_left = self.sdk_dll_handle._7i_set_data(b'biLeft', len(b'biLeft'), local_left_coe.buf, left_buf_len)
        ret_right = self.sdk_dll_handle._7i_set_data(b'biRight', len(b'biRight'), local_right_coe.buf, right_buf_len)
        if ret_left != 0 or ret_right != 0:
            raise RuntimeError("failed to load calibration profile: L=%d R=%d" % (ret_left, ret_right))

    def stop(self):
        ret = self.sdk_dll_handle._7i_stop()
        print('_7i_stop:%d' % ret)
        self.h256_dll_handle._7i_h265_release()

    def start_calibration(self, points):
        print('enter start_calibration')
        if not self.thread_is_running:
            self.calib_points = points
            self.flag_exit_thread = False
            self.thread_handle = threading.Thread(target=self.calibration_thread_func)
            self.thread_handle.start()
        print('leave start_calibration')

    def cancel_current_calibration_point(self):
        self.sdk_dll_handle._7i_cancel_calibration(PY_7I_EYE_TYPE.L_EYE.value)
        self.sdk_dll_handle._7i_cancel_calibration(PY_7I_EYE_TYPE.R_EYE.value)

    def stop_calibration(self):
        print('enter complete_calibration')
        if self.thread_is_running:
            self.flag_exit_thread = True
            wrapper.select_Point_event.set()
            self.thread_handle.join()
        print('leave complete_calibration')

    def calibration_thread_func(self):
        self.thread_is_running = True
        ret = self.sdk_dll_handle._7i_start_calibration(self.calib_points)
        print("_7i_start_calibration: %d" % ret)

        index = 0
        for i in range(index, self.calib_points):
            wrapper.select_Point_event.clear()
            wrapper.select_Point_event.wait()  # Wait to select the calibration point event on the screen
            if self.flag_exit_thread:
                break  # Exit for

            index = (i + 1)
            pt = py_7i_point2d_t()
            pt.x = self.cur_point_x
            pt.y = self.cur_point_y

            ret1 = self.sdk_dll_handle._7i_start_calibration_point(PY_7I_EYE_TYPE.L_EYE.value, index, pointer(pt),
                                                                   self.py_left_point_process_cb, py_object(self),
                                                                   self.py_left_point_finish_cb, py_object(self))

            ret2 = self.sdk_dll_handle._7i_start_calibration_point(PY_7I_EYE_TYPE.R_EYE.value, index, pointer(pt),
                                                                   self.py_right_point_process_cb, py_object(self),
                                                                   self.py_right_point_finish_cb, py_object(self))

            print("_7i_start_calibration_point: %d %d" % (ret1, ret2))

        time.sleep(3)  # Wait for the last calibration point to complete，
        # In addition, a better approach is to control based on the state of the finish callback,
        # That would be more perfect !!!

        left_coe = py_7i_coefficient_t()
        right_coe = py_7i_coefficient_t()
        ret1 = self.sdk_dll_handle._7i_compute_calibration(PY_7I_EYE_TYPE.L_EYE.value, pointer(left_coe))
        ret2 = self.sdk_dll_handle._7i_compute_calibration(PY_7I_EYE_TYPE.R_EYE.value, pointer(right_coe))
        print("_7i_compute_calibration: %d %d" % (ret1, ret2))
        if 0 == ret1 and 0 == ret2:
            left_buf_len = c_int(1024)
            flag_left_coe = b'biLeft'
            print(len(flag_left_coe))
            ret1 = self.sdk_dll_handle._7i_get_data(flag_left_coe, len(flag_left_coe), left_coe.buf, byref(left_buf_len))
            print("_7i_get_data: L %d %d" % (ret1, left_buf_len.value))

            right_buf_len = c_int(1024)
            flag_right_coe = b'biRight'
            ret2 = self.sdk_dll_handle._7i_get_data(flag_right_coe, len(flag_right_coe), right_coe.buf, byref(right_buf_len))
            print("_7i_get_data: R %d %d" % (ret2, right_buf_len.value))

            score_buf_len = c_int(64)
            flag_score = b'get_score'            
            score_buf = (c_char * 64)()                     
            ret3 = self.sdk_dll_handle._7i_get_data(flag_score, len(flag_score), score_buf, byref(score_buf_len))
            print("_7i_get_data: S %d %d" % (ret3, score_buf_len.value))
            print(score_buf.value.decode('utf-8'))

        ret = self.sdk_dll_handle._7i_complete_calibration()
        print("_7i_complete_calibration: %d" % ret)

        if 0 == ret1 and 0 == ret2:
            ret1 = 0 #self.sdk_dll_handle._7i_start_tracking(PY_7I_EYE_TYPE.L_EYE.value, pointer(left_coe))
            ret2 = 0 #self.sdk_dll_handle._7i_start_tracking(PY_7I_EYE_TYPE.R_EYE.value, pointer(right_coe))
            #print("_7i_start_tracking: %d %d" % (ret1, ret2))
            left_buf_len = c_int(1024)
            flag_left_coe = b'biLeft'
            right_buf_len = c_int(1024)
            flag_right_coe = b'biRight'
            ret1 = self.sdk_dll_handle._7i_set_data(flag_left_coe, len(flag_left_coe), left_coe.buf, left_buf_len)
            ret2 = self.sdk_dll_handle._7i_set_data(flag_right_coe, len(flag_right_coe), right_coe.buf, right_buf_len)
            print("_7i_set_data: L %d" % (ret1))

        self.thread_is_running = False

    def start_tracking(self):        
        local_left_coe = py_7i_coefficient_t()
        with open('./left_coe.dat', 'rb') as fp:
            data = fp.read()
            memmove(local_left_coe.buf, data, 1024)
            #print("read left coe")
        fp.close()
       
        local_right_coe = py_7i_coefficient_t()
        with open('./right_coe.dat', 'rb') as fp:
            data = fp.read()
            memmove(local_right_coe.buf, data, 1024)
            #print("read right coe")
        fp.close()

        left_buf_len = c_int(1024)
        flag_left_coe = b'biLeft'
        right_buf_len = c_int(1024)
        flag_right_coe = b'biRight'
        ret1 = self.sdk_dll_handle._7i_set_data(flag_left_coe, len(flag_left_coe), local_left_coe.buf, left_buf_len)
        ret2 = self.sdk_dll_handle._7i_set_data(flag_right_coe, len(flag_right_coe), local_right_coe.buf, right_buf_len)
        print("_7i_set_data: L %d" % (ret1))
        print("_7i_set_data: R %d" % (ret2))

        # Notes:
        # If you want to save the calibration coefficients as a file or
        # read the calibration coefficients from a local file, you can use the code below.
        # Save the coefficients as a file ==============================================================================
        # if 0 == ret1:
        #     res = bytearray(1024)
        #     ptr = (c_ubyte * 1024).from_buffer(res)
        #     memmove(ptr, left_coe.buf, 1024)
        #     with open('./left_coe.dat', 'wb') as fp:
        #         for i in ptr:
        #             s = struct.pack('B', i)
        #             fp.write(s)
        #     fp.close()
        #
        # if 0 == ret2:
        #     res = bytearray(1024)
        #     ptr = (c_ubyte * 1024).from_buffer(res)
        #     memmove(ptr, right_coe.buf, 1024)
        #     with open('./right_coe.dat', 'wb') as fp:
        #         for i in ptr:
        #             s = struct.pack('B', i)
        #             fp.write(s)
        #     fp.close()

        #  Read coefficients from a local file =========================================================================
        # local_left_coe = py_7i_coefficient_t()
        # with open('./left_coe.dat', 'rb') as fp:
        #     data = fp.read()
        #     memmove(local_left_coe.buf, data, 1024)
        # fp.close()
        #
        # local_right_coe = py_7i_coefficient_t()
        # with open('./right_coe.dat', 'rb') as fp:
        #     data = fp.read()
        #     memmove(local_right_coe.buf, data, 1024)
        # fp.close()
