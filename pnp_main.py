#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, DUKELEC, Inc.
# All rights reserved.
#
# Author: Duke Fong <d@d-l.io>

"""CDPNP GUI tool

"""

import sys, os, subprocess
import struct
from time import sleep
import asyncio, qasync, _thread
import math
import numpy as np
import cv2 as cv
from scipy.optimize import fsolve
from cd_ws import CDWebSocket, CDWebSocketNS
from web_serve import ws_ns, start_web

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtWebEngineWidgets import *

sys.path.append(os.path.join(os.path.dirname(__file__), 'pycdnet'))

from cdnet.utils.log import *
from cdnet.utils.cd_args import CdArgs
from cdnet.utils.serial_get_port import *
from cdnet.dev.cdbus_serial import CDBusSerial
from cdnet.dispatch import *

from pnp_cv import pnp_cv_init, cv_dat, cur_path
from pnp_xyz import *

args = CdArgs()
dev_str = args.get("--dev", dft=":5740")
resource_path = os.path.join(cur_path, 'resource')
utils_path = os.path.join(cur_path, 'utils')

if args.get("--help", "-h") != None:
    print(__doc__)
    exit()

if args.get("--verbose", "-v") != None:
    logger_init(logging.VERBOSE)
elif args.get("--debug", "-d") != None:
    logger_init(logging.DEBUG)
elif args.get("--info", "-i") != None:
    logger_init(logging.INFO)

logging.getLogger('websockets').setLevel(logging.WARNING)
logger = logging.getLogger(f'cdpnp')

if dev_str == ':5740':
    if get_port('2E3C:5740'):
        dev_str = '2E3C:5740' # bridge hw v6
    elif get_port('0483:5740'):
        dev_str = '0483:5740' # bridge hw v5
    else:
        logger.error(f'cdbus bridge not found, serial ports:')
        dump_ports()
        exit()

# baudrate ignored for cdbus bridge
dev = CDBusSerial(dev_str) if dev_str != 'None' else None
if dev:
    CDNetIntf(dev, mac=0x00)
    xyz_init()

print('start...')


coeff = None
fiducial_pcb = [ [0, 0], [1, 1] ]
fiducial_cam = [ [0, 0], [10, 10] ]

def equations(p):
    s, a, d_x, d_y = p
    F = np.empty((4))
    for i in range(2):
        F[i*2] = (fiducial_pcb[i][0] * math.cos(a) - fiducial_pcb[i][1] * math.sin(a)) * s + d_x - fiducial_cam[i][0]
        F[i*2+1] = (fiducial_pcb[i][0] * math.sin(a) + fiducial_pcb[i][1] * math.cos(a)) * s + d_y - fiducial_cam[i][1]
    return F

def pcb2xyz(p, pcb):
    s, a, d_x, d_y = p
    step_x = (pcb[0] * math.cos(a) - pcb[1] * math.sin(a)) * s + d_x
    step_y = (pcb[0] * math.sin(a) + pcb[1] * math.cos(a)) * s + d_y
    return step_x, step_y, math.degrees(math.atan2(math.sin(-a), math.cos(-a))) # return limited-range angle

def update_coeff():
    global coeff
    coeff = fsolve(equations, (1, 1, 1, 1)) # ret: scale, angle, del_x, del_y
    #print(f'coefficient:', coeff)
    #print('equations(coeff):', equations(coeff))
    #print('coeff:', coeff)


async def dev_service():
    global coeff, fiducial_pcb, fiducial_cam

    sock = CDWebSocket(ws_ns, 'dev')
    while True:
        dat, src = await sock.recvfrom()
        logger.debug(f'dev ser: {dat}')

        if dat['action'] == 'get_motor_pos':
            logger.info('get_motor_pos')
            p = load_pos() if dev else [0, 0, 0, 0]
            await sock.sendto(p, src)

        elif dat['action'] == 'set_motor_pos':
            logger.info('set_motor_pos')
            if dev:
                goto_pos(dat['pos'], dat['wait'], dat['speed'])
            await sock.sendto('succeeded', src)


        elif dat['action'] == 'get_pump_hw_ver':
            logger.info(f"get_pump_hw_ver: {xyz['pump_hw_ver']}")
            await sock.sendto(xyz['pump_hw_ver'], src)

        elif dat['action'] == 'get_pump_pressure':
            logger.info(f"get_pump_pressure")
            pressure = 0.0
            if dev:
                pressure = get_pump_pressure()
            await sock.sendto(pressure, src)

        elif dat['action'] == 'set_pump':
            logger.info(f"set_pump {dat['val']}")
            if dev:
                set_pump(dat['val'])
            await sock.sendto('succeeded', src)

        elif dat['action'] == 'set_camera':
            logger.info(f"set_camera {dat['val']}")
            if dev:
                rx = cd_reg_rw(f"80:00:2{cv_dat['dev']}", 0x005f, struct.pack("<B", 255 if dat['val'] else 0))
                print('set cam ret: ' + rx.hex())
            await sock.sendto('succeeded', src)

        elif dat['action'] == 'set_camera_cfg':
            logger.info(f"set_camera_cfg dev: {dat['dev']}, detect: {dat['detect']}, light: {dat['light1']}_{dat['light2']}, expos: {dat['expos']}")
            cv_dat['dev'] = dat['dev']
            cv_dat['detect'] = dat['detect']
            rx = cd_reg_rw(f"80:00:21", 0x0069, struct.pack("<b", 1 if dat['light1'] else 0))
            print('set cam_light1 ret: ' + rx.hex())
            rx = cd_reg_rw(f"80:00:22", 0x0069, struct.pack("<b", 1 if dat['light2'] else 0))
            print('set cam_light2 ret: ' + rx.hex())
            rx = cd_reg_rw(f"80:00:2{cv_dat['dev']}", 0x0044, struct.pack("<H", dat['expos']))
            print('set exposure ret: ' + rx.hex())
            await sock.sendto('succeeded', src)

        elif dat['action'] == 'get_camera_cfg':
            logger.info('get_camera_cfg')
            rx0 = cd_reg_rw(f"80:00:2{cv_dat['dev']}", 0x005f, read=1) if dev else bytes([0x80, 0x00])
            print('get_camera_cfg ret: ' + rx0.hex())
            rx1 = cd_reg_rw(f"80:00:21", 0x0069, read=1) if dev else bytes([0x80, 0x00])
            print('get_camera_light1 ret: ' + rx1.hex())
            rx2 = cd_reg_rw(f"80:00:22", 0x0069, read=1) if dev else bytes([0x80, 0x00])
            print('get_camera_light2 ret: ' + rx2.hex())
            await sock.sendto({'enable': rx0[1], 'dev': cv_dat['dev'], 'detect': cv_dat['detect'], 'light1': rx1[1], 'light2': rx2[1]}, src)

        elif dat['action'] == 'set_vision_cfg':
            logger.info(f"set_vision_cfg nozzle_thresh: {dat['nozzle_thresh']}, debug: {dat['debug']}")
            cv_dat['nozzle_thresh'] = dat['nozzle_thresh']
            cv_dat['debug'] = dat['debug']
            await sock.sendto('succeeded', src)

        elif dat['action'] == 'update_camera_bg':
            logger.info(f"update_camera_bg...")
            cv_dat['bg_capture'] = True
            await sock.sendto('succeeded', src)

        elif dat['action'] == 'remove_camera_bg':
            logger.info(f"remove_camera_bg...")
            cv_dat['bg_img'] = None
            if os.path.exists(f'{cur_path}/tmp/bg_invert.png'):
                os.remove(f'{cur_path}/tmp/bg_invert.png')
            await sock.sendto('succeeded', src)

        elif dat['action'] == 'pcb2xyz':
            logger.info(f"pcb2xyz")
            fiducial_pcb = dat['pcb']
            fiducial_cam = dat['cam']
            update_coeff()
            p_xy = pcb2xyz(coeff, (dat['x'], dat['y']))
            await sock.sendto(p_xy, src)

        elif dat['action'] == 'get_cv_cur':
            logger.info(f"get_cv_cur")
            await sock.sendto(cv_dat['cur'], src)

        elif dat['action'] == 'wait_stop':
            logger.info(f"wait_stop")
            if dev:
                wait_stop()
            await sock.sendto('succeeded', src)

        elif dat['action'] == 'enable_force':
            logger.info(f"enable_force")
            if dev:
                enable_force()
            await sock.sendto('succeeded', src)

        else:
            await sock.sendto('err: dev: unknown cmd', src)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CDPNP Controller")
        self.setFixedSize(2560, 1440)
        self.init_ui()

    def init_ui(self):
        """初始化UI布局"""
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # 左侧主面板
        self.init_browser(main_layout)

        # 右侧面板
        self.init_right_panel(main_layout)

        # 定时器，用于周期性更新图像
        self.init_timer()

    def init_browser(self, main_layout):
        """初始化浏览器部分"""
        self.browser = QWebEngineView()
        self.browser.load(QUrl("http://localhost:8900"))
        self.browser.setZoomFactor(2.0)

        # 禁用缩放功能
        # settings = self.browser.settings()
        # settings.setAttribute(QWebEngineSettings.WebAttribute.FullScreenSupportEnabled, False)
        # settings.setAttribute(QWebEngineSettings.WebAttribute.PluginsEnabled, False)

        main_layout.addWidget(self.browser, stretch=3)

    def init_right_panel(self, main_layout):
        """初始化右侧面板"""
        right_panel = QWidget()
        right_panel.setStyleSheet("background-color: #f0f0f0;")
        right_layout = QVBoxLayout(right_panel)

        # 图像显示部分
        self.init_image_panel(right_layout)

        # 控制部分
        self.init_control_panel(right_layout)

        main_layout.addWidget(right_panel, stretch=1)

    def init_image_panel(self, right_layout):
        """初始化图像显示部分"""
        image_panel = QWidget()
        image_panel.setStyleSheet("background-color: #ffffff;")
        image_panel.setFixedSize(600, 800)
        self.image_label = QLabel("Image Loading...")
        self.image_label.setAlignment(Qt.AlignCenter)

        image_layout = QVBoxLayout(image_panel)
        image_layout.addWidget(self.image_label)

        right_layout.addWidget(image_panel, stretch=2)

    def init_control_panel(self, right_layout):
        """初始化控制部分"""
        control_panel = QWidget()
        control_panel.setStyleSheet("background-color: #e0e0e0;")
        control_layout = QVBoxLayout(control_panel)

        # 控制面板标题
        control_label = QLabel("Control Loading...")
        control_label.setAlignment(Qt.AlignCenter)
        control_layout.addWidget(control_label)

        # 添加功能按钮
        self.init_buttons(control_layout)

        right_layout.addWidget(control_panel, stretch=1)

    def init_buttons(self, control_layout):
        """初始化功能按钮"""
        # 立创坐标转换按钮
        self.run_script_button = QPushButton("LCEDA坐标转换")
        self.run_script_button.clicked.connect(self.run_python_script)
        control_layout.addWidget(self.run_script_button)

        # 急停按钮
        self.stop_button = QPushButton("STOP")
        self.stop_button.clicked.connect(self.motor_stop)
        control_layout.addWidget(self.stop_button)

    def init_timer(self):
        """初始化定时器"""
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(10)

    def update_image(self):
        """从队列中获取图像并显示"""
        if cv_dat['local'] and cv_dat['init'] and not cv_dat['img_queue'].empty():
            cur_pic = cv_dat['img_queue'].get()
            cur_pic = cv.cvtColor(cur_pic, cv.COLOR_BGR2RGB)
            if cv_dat['dev'] == 2:
                cur_pic = cv.rotate(cur_pic, cv.ROTATE_90_COUNTERCLOCKWISE)
            height, width, chanel = cur_pic.shape
            if height == 800 and width == 600:
                bytes_per_line = 3 * width
                q_img = QImage(cur_pic.data, width, height, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(q_img)
                self.image_label.setPixmap(pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio))

    def run_python_script(self):
        """执行 Python 脚本"""
        try:
            file_path, _ = QFileDialog.getOpenFileName(None, "选择CSV文件", "", "CSV文件 (*.csv)")
            if file_path:
                script_path = os.path.join(utils_path, 'parse_csv_lceda.py')
                subprocess.run(['python3', script_path, file_path], check=True)
                logger.info("Python 脚本执行成功")
            else:
                logger.info("未选择CSV文件")
        except subprocess.CalledProcessError as e:
            logger.error(f"Python 脚本执行失败: {e}")

    def motor_stop(self):
        print("stop")

async def pyqt_service():
    window = MainWindow()
    window.show()
    window.browser.reload()  # 刷新页面确保连接
    # 使用QTimer保持Qt事件循环运行
    timer = QTimer()
    timer.timeout.connect(lambda: None)  # 空函数，仅用于保持事件循环
    timer.start(100)  # 每100毫秒触发一次

    # 保持主线程运行
    while True:
        await asyncio.sleep(1)  # 保持异步事件循环运行

def main_loop():
    pnp_app = QApplication(sys.argv)
    # 设置应用程序图标
    icon_path = os.path.join(resource_path, 'pnp_icon.svg')  # 图标文件路径
    if os.path.exists(icon_path):
        pnp_app.setWindowIcon(QIcon(icon_path))
    loop = qasync.QEventLoop(pnp_app)
    asyncio.set_event_loop(loop)
    try:
        # 启动任务
        tasks = [
            loop.create_task(start_web()),
            loop.create_task(dev_service()),
            loop.create_task(pyqt_service())
        ]

        # 监听应用退出事件
        def on_quit():
            logger.info("正在关闭应用...")
            print("正在关闭应用...")
            for task in tasks:
                task.cancel()  # 取消所有异步任务
            loop.stop()  # 停止事件循环
            _thread.exit()  # 终止所有线程

        pnp_app.aboutToQuit.connect(on_quit)
        loop.run_until_complete(asyncio.gather(*tasks))
        loop.run_forever()
    except Exception as e:
        logger.error(f"程序终止: {str(e)}")
    finally:
        loop.close()
        _thread.exit()  # 终止所有线程
        sys.exit(1)

if __name__ == "__main__":

    if dev:
        _thread.start_new_thread(main_loop, ())
        pnp_cv_init() # make cv gui in foreground thread (for macos)
        while True:
            sleep(5)
    else:
        main_loop()
