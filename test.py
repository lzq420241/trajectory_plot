import asyncio
import bleak
import device_model
import plotter
import tracker
import threading
from matplotlib import pyplot as plt

# 扫描到的设备 Scanned devices
devices = []
# 蓝牙设备 BLEDevice
BLEDevice = None


vis = plotter.RealtimeTrajectoryVisualizer(max_points=300)


# 定义统一事件回调
def handle_event(event: tracker.TrajectoryEvent):
    vis.add_event(event)


my_tracker = tracker.RealtimeTrajectoryTracker(
    window_size=100,       # 1秒检测窗口
    acc_var_thresh=0.2,    # 加速度方差阈值
    gyro_thresh=3.0,       # 角速度阈值（deg/s）
    zupt_alpha=0.05,        # 零偏更新速度
    event_callback=handle_event
)


# 扫描蓝牙设备并过滤名称
# Scan Bluetooth devices and filter names
async def scan():
    global devices
    global BLEDevice
    find = []
    print("Searching for Bluetooth devices......")
    try:
        devices = await bleak.BleakScanner.discover(timeout=20.0)
        print("Search ended")
        for d in devices:
            if d.name is not None and "WT" in d.name:
                find.append(d)
                print(d)
        if len(find) == 0:
            print("No devices found in this search!")
        else:
            user_input = input("Please enter the Mac address you want to connect to (e.g. DF:E9:1F:2C:BD:59)：")
            for d in devices:
                if d.address == user_input:
                    BLEDevice = d
                    break
    except Exception as ex:
        print("Bluetooth search failed to start")
        print(ex)


# 指定MAC地址搜索并连接设备
# Specify MAC address to search and connect devices
async def scanByMac(device_mac):
    global BLEDevice
    print("Searching for Bluetooth devices......")
    BLEDevice = await bleak.BleakScanner.find_device_by_address(device_mac, timeout=20)


# 数据更新时会调用此方法 This method will be called when data is updated
def updateData(DeviceModel):
    # 直接打印出设备数据字典 Directly print out the device data dictionary
    d = DeviceModel.deviceData

    # my_tracker.process_frame(d['AccRaw'], d['GyroRaw'], d['EulerAngles'])
    my_tracker.process_frame([d['PosE'], d['PosN'], d['PosU']])
    # print(d)
    # 获得X轴加速度 Obtain X-axis acceleration
    # print(DeviceModel.get("AccX"))


async def main():
    # 方式一：广播搜索和连接蓝牙设备
    # Method 1:Broadcast search and connect Bluetooth devices
    # await scan()

    # # 方式二：指定MAC地址搜索并连接设备
    # # Method 2: Specify MAC address to search and connect devices
    await scanByMac("779CDA46-44D7-4C5C-8730-FDE639589FC7")

    if BLEDevice is not None:
        # 创建设备 Create device
        device = device_model.DeviceModel("MyBle5.0", BLEDevice, updateData)
        # 开始连接设备 Start connecting devices
        await device.openDevice()
    else:
        print("This BLEDevice was not found!!")


def async_main():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(main())
    finally:
        loop.close()


if __name__ == '__main__':
    # 启动异步任务线程
    ble_thread = threading.Thread(target=async_main)
    ble_thread.daemon = True  # 设置为守护线程防止阻塞退出
    ble_thread.start()

    # 主线程运行Matplotlib
    plt.show(block=True)
