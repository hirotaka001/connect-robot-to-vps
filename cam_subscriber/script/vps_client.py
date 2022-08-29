import os, datetime, random
import string, json
import requests
import threading
import numpy as np
import cv2
import datetime
import time

from scipy.spatial.transform import Rotation as R

dt_now = datetime.datetime.now()

class VpsUtil:
    @staticmethod
    def c_timestamp():
        d_time = datetime.datetime.now()
        time_stamp = d_time.strftime('%Y-%m-%d %H:%M:%S')
        return time_stamp

    @staticmethod
    def randomname(n):
        return ''.join(random.choices(string.ascii_letters + string.digits, k=n))


class VpsParam:
    def __init__(self, url, map_id, focal_length, intrinsic_file, work_dir):
        self._vps_url = url
        self._vps_map_id = map_id
        self._focal_length = focal_length
        self._max_w = 640
        self._work_dir = work_dir
        self._intrinsic_param_file_path = intrinsic_file

class VpsClient(threading.Thread):
    def __init__(self, vps_param):
        super(VpsClient, self).__init__()
        self.__vps_param = vps_param
        self.daemon = True
        # TODO change to infinity
        self.__loop_max = 99999
        return

    def close(self):
        print('loop max is 0')
        self.__loop_max = 0

    def run(self, vps_queue, callback, user_data):
        print('vps worker start!')
        t = threading.Thread(target=self._vps_worker, args=(vps_queue, callback, user_data))
        t.setDaemon(True)
        t.start()
        return 

    # 
    def _check_result(self, res):
        print("res[\"status\"]",res["status"])
        if res["status"] == 0 or res["position"][1] < 0.9 or res["position"][1] > 1.5:
            # 不明、もしくは誤り
            print("VPS Failed")
            return False
        else:
            # その他のエリア
            print("VPS Success")
            return True

    def _preprocess_image(self, img, working_dir, code_path, max_w, loop_count):
        if True: # 歪み補正あり画像送信
            org = img
            _img = img

            # # trim 
            #height, width = img.shape[:2]
            #_img = img[int(height/4) : int(height*3/4), int(width/4) : int(width*3/4)]
            # print('trim img size:', _img.shape[:2])

            mtx = np.load(code_path + 'mtx.npy')
            dist = np.load(code_path + 'dist.npy')           
            h,  w = _img.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
            # 歪補正
            dst = cv2.undistort(_img, mtx, dist, None, newcameramtx)
            # 画像の切り落とし
            x,y,w,h = roi
            _dst = dst[y:y+h, x:x+w]
            img = _dst
            
            if 0:
                cv2.imshow('img',img)
                cv2.waitKey(500)
                cv2.imshow('_dst',_dst)
                cv2.waitKey(500)
  
            now = dt_now.strftime('%Y%m%d')
            image_path = "{}/{}{}{}.{}".format(working_dir, "send_cut_", now, str(loop_count), "jpg")
            cv2.imwrite(image_path, _dst)
            if 1:
                save_image_path = "{}/{}{}{}.{}".format(working_dir, "send_", now, str(loop_count), "jpg")
                #cv2.imwrite(save_image_path, dst)
                org_image_path = "{}/{}{}{}.{}".format(working_dir, "org_", now, str(loop_count), "jpg")
                cv2.imwrite(org_image_path, org)
        else: # distortion is not correlate. (for iPhone)
            now = dt_now.strftime('%Y%m%d')
            image_path = "{}/{}{}{}.{}".format(working_dir, "send_cut_", now, str(loop_count), "jpg")
            cv2.imwrite(image_path, img)

        # if over max size, image is resized
        if img.shape[1] > max_w:
            print("image size is larger than ", max_w, ". resized")
            new_w = max_w
            new_h = int(max_w*img.shape[0]/img.shape[1])

            img_small = cv2.resize(img , (new_w, new_h))

            cv2.imwrite("tmp/" + os.path.basename(image_path), img_small)

            image = open("tmp/" + os.path.basename(image_path), 'rb')
            files = {'qimage': (os.path.basename(image_path), image, 'image/jpeg')}
        else:
            image = open(image_path, 'rb')
            files =  {'qimage': (os.path.basename(image_path), image, 'image/jpeg')}

        return files

    def _vps_worker(self, vps_queue, callback, user_data):
        working_dir = self.__vps_param._work_dir

        code_path = self.__vps_param._intrinsic_param_file_path
        url = self.__vps_param._vps_url
        map_id = self.__vps_param._vps_map_id
        
        loop_count = 0

        while self.__loop_max > loop_count:           
            if vps_queue.qsize() == 0:
                continue

            #print('LoopMax', self.__loop_max, loop_count)
            loop_count = loop_count + 1
            vps_data = vps_queue.get()
            time_stamp, img, mes_id = vps_data

            files = self._preprocess_image(img, working_dir, code_path, self.__vps_param._max_w, loop_count)

            if self.__vps_param._focal_length > 0:
                data = {'message_id': mes_id,'FocalLengthIn35mmFilm': self.__vps_param._focal_length, 'map_id': map_id}
            else:
                data = {'message_id': mes_id, 'map_id': map_id}

            #POST送信
            try:
                response = requests.post(url, data = data, files = files)
            except Exception as e:
                print("ERROR :", e)
            else:
                v_res = json.loads(response.text)
                callback(v_res, user_data)
                if v_res['status'] == 0:
                    print('return response from vps. but estimation of position was failed')

            # for debug
            # cv2.imshow('test img', img)
            # cv2.waitKey(0)


        print('loop count over. exit vps process')
        exit(1)


