# coding:UTF-8
import time
import bleak
import asyncio
import numpy as np


# 设备实例 Device instance
class DeviceModel:
    # region 属性 attribute
    # 设备名称 deviceName
    deviceName = "我的设备"

    # 设备数据字典 Device Data Dictionary
    deviceData = {}

    # 设备是否开启
    isOpen = False

    # 临时数组 Temporary array
    TempBytes = []

    # endregion

    def __init__(self, deviceName, BLEDevice, callback_method):
        print("Initialize device model")
        # 设备名称（自定义） Device Name
        self.deviceName = deviceName
        self.BLEDevice = BLEDevice
        self.client = None
        self.writer_characteristic = None
        self.isOpen = False
        self.callback_method = callback_method
        self.deviceData = {}

    # region 获取设备数据 Obtain device data
    # 设置设备数据 Set device data
    def set(self, key, value):
        # 将设备数据存到键值 Saving device data to key values
        self.deviceData[key] = value

    # 获得设备数据 Obtain device data
    def get(self, key):
        # 从键值中获取数据，没有则返回None Obtaining data from key values
        if key in self.deviceData:
            return self.deviceData[key]
        else:
            return None

    # 删除设备数据 Delete device data
    def remove(self, key):
        # 删除设备键值
        del self.deviceData[key]

    # endregion
    @staticmethod
    def rpy_to_quaternion(roll, pitch, yaw):
        # 转换角度为弧度
        return Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=True).as_quat()

    # 打开设备 open Device
    async def openDevice(self):
        print("Opening device......")
        # 获取设备的服务和特征 Obtain the services and characteristic of the device
        async with bleak.BleakClient(self.BLEDevice, loop=asyncio.get_running_loop(), timeout=15) as client:
            self.client = client
            self.isOpen = True
            # 设备UUID常量 Device UUID constant
            target_service_uuid = "0000ffe5-0000-1000-8000-00805f9a34fb"
            target_characteristic_uuid_read = "0000ffe4-0000-1000-8000-00805f9a34fb"
            target_characteristic_uuid_write = "0000ffe9-0000-1000-8000-00805f9a34fb"
            notify_characteristic = None

            print("Matching services......")
            for service in client.services:
                if service.uuid == target_service_uuid:
                    print(f"Service: {service}")
                    print("Matching characteristic......")
                    for characteristic in service.characteristics:
                        if characteristic.uuid == target_characteristic_uuid_read:
                            notify_characteristic = characteristic
                        if characteristic.uuid == target_characteristic_uuid_write:
                            self.writer_characteristic = characteristic
                    if notify_characteristic:
                        break
            # 设置输出内容为 位移+位移速度+时间戳
            await self.writeReg(0x96, 0x0003)

            # if self.writer_characteristic:
            #     # 读取电量
            #     print("Reading battery level")
            #     time.sleep(3)
            #     asyncio.create_task(self.sendDataTh())

            if notify_characteristic:
                print(f"Characteristic: {notify_characteristic}")
                # 设置通知以接收数据 Set up notifications to receive data
                await client.start_notify(notify_characteristic.uuid, self.onDataReceived)

                # 保持连接打开 Keep connected and open
                try:
                    while self.isOpen:
                        await asyncio.sleep(1)
                except asyncio.CancelledError:
                    pass
                finally:
                    # 在退出时停止通知 Stop notification on exit
                    await client.stop_notify(notify_characteristic.uuid)
            else:
                print("No matching services or characteristic found")

    # 关闭设备  close Device
    def closeDevice(self):
        self.isOpen = False
        print("The device is turned off")

    async def sendDataTh(self):
        while self.isOpen:
            # 读取电量
            # await self.readReg(0x64)
            time.sleep(5)

    # region 数据解析 data analysis
    # 串口数据处理  Serial port data processing
    def onDataReceived(self, sender, data):
        tempdata = bytes.fromhex(data.hex())
        for var in tempdata:
            self.TempBytes.append(var)
            if len(self.TempBytes) == 1 and self.TempBytes[0] != 0x55:
                del self.TempBytes[0]
                continue
            if len(self.TempBytes) == 2 and (self.TempBytes[1] != 0x61 and self.TempBytes[1] != 0x71):
                del self.TempBytes[0]
                continue
            if len(self.TempBytes) == 20:
                self.processData(self.TempBytes)
                self.TempBytes.clear()

    @staticmethod
    def register_to_percentage(register_value):
        # 寄存器值范围
        if register_value > 396:
            return 100
        elif 393 <= register_value <= 396:
            return 90
        elif 387 <= register_value < 393:
            return 75
        elif 382 <= register_value < 387:
            return 60
        elif 379 <= register_value < 382:
            return 50
        elif 377 <= register_value < 379:
            return 40
        elif 373 <= register_value < 377:
            return 30
        elif 370 <= register_value < 373:
            return 20
        elif 368 <= register_value < 370:
            return 15
        elif 350 <= register_value < 368:
            return 10
        elif 340 <= register_value < 350:
            return 5
        elif register_value < 340:
            return 0
        else:
            return "Invalid register value"

    # 数据解析 data analysis
    def processData(self, Bytes):
        if Bytes[1] == 0x61:
            # Ax = self.getSignInt16(Bytes[3] << 8 | Bytes[2]) / 32768 * 16
            # Ay = self.getSignInt16(Bytes[5] << 8 | Bytes[4]) / 32768 * 16
            # Az = self.getSignInt16(Bytes[7] << 8 | Bytes[6]) / 32768 * 16
            # Gx = self.getSignInt16(Bytes[9] << 8 | Bytes[8]) / 32768 * 2000
            # Gy = self.getSignInt16(Bytes[11] << 8 | Bytes[10]) / 32768 * 2000
            # Gz = self.getSignInt16(Bytes[13] << 8 | Bytes[12]) / 32768 * 2000
            # AngR = self.getSignInt16(Bytes[15] << 8 | Bytes[14]) / 32768 * 180
            # AngP = self.getSignInt16(Bytes[17] << 8 | Bytes[16]) / 32768 * 180
            # AngY = self.getSignInt16(Bytes[19] << 8 | Bytes[18]) / 32768 * 180
            # Qx, Qy, Qz, Qw = self.rpy_to_quaternion(AngX, AngY, AngZ)

            # self.set("AccRaw", np.array([Ax, Ay, Az]))
            # self.set("GyroRaw", np.array([Gx, Gy, Gz]))
            # self.set("EulerAngles", np.array([AngY, AngP, AngR]))
            PosE = self.getSignInt16(Bytes[3] << 8 | Bytes[2])
            PosN = self.getSignInt16(Bytes[5] << 8 | Bytes[4])
            PosU = self.getSignInt16(Bytes[7] << 8 | Bytes[6])
            VelE = self.getSignInt16(Bytes[9] << 8 | Bytes[8])
            VelN = self.getSignInt16(Bytes[11] << 8 | Bytes[10])
            VelU = self.getSignInt16(Bytes[13] << 8 | Bytes[12])
            current_time = Bytes[17] << 24 | Bytes[16] << 16 | Bytes[15] << 8 | Bytes[14]
            # test = Bytes[19] << 8 | Bytes[18]
            # # xyz 位移
            self.set("PosE", PosE)
            self.set("PosN", PosN)
            self.set("PosU", PosU)
            # # xyz 速度
            self.set("VelE", VelE)
            self.set("VelN", VelN)
            self.set("VelU", VelU)
            # self.set("Test", hex(test))
            # # rpy 角度
            # self.set("AngX", round(AngX, 3))
            # self.set("AngY", round(AngY, 3))
            # self.set("AngZ", round(AngZ, 3))
            #  = time.time()
            # formatted_time = time.strftime("%Y-%m-%d %H:%M:%S",
            #                                time.localtime(current_time)) + f".{int(current_time * 1000) % 1000:03d}"
            self.set("timestamp", hex(current_time))
            self.callback_method(self)
        else:
            # 磁场 magnetic field
            if Bytes[2] == 0x3A:
                Hx = self.getSignInt16(Bytes[5] << 8 | Bytes[4]) / 120
                Hy = self.getSignInt16(Bytes[7] << 8 | Bytes[6]) / 120
                Hz = self.getSignInt16(Bytes[9] << 8 | Bytes[8]) / 120
                self.set("HX", round(Hx, 3))
                self.set("HY", round(Hy, 3))
                self.set("HZ", round(Hz, 3))
            # 电量 Battery level
            elif Bytes[2] == 0x64:
                battery_voltage = self.getSignInt16(Bytes[5] << 8 | Bytes[4])
                self.set("battery_lvl", self.register_to_percentage(battery_voltage))
            # 四元数 Quaternion
            elif Bytes[2] == 0x51:
                Q0 = self.getSignInt16(Bytes[5] << 8 | Bytes[4]) / 32768
                Q1 = self.getSignInt16(Bytes[7] << 8 | Bytes[6]) / 32768
                Q2 = self.getSignInt16(Bytes[9] << 8 | Bytes[8]) / 32768
                Q3 = self.getSignInt16(Bytes[11] << 8 | Bytes[10]) / 32768
                self.set("Q0", round(Q0, 5))
                self.set("Q1", round(Q1, 5))
                self.set("Q2", round(Q2, 5))
                self.set("Q3", round(Q3, 5))
            else:
                pass

    # 获得int16有符号数 Obtain int16 signed number
    @staticmethod
    def getSignInt16(num):
        if num >= pow(2, 15):
            num -= pow(2, 16)
        return num

    # endregion

    # 发送串口数据 Sending serial port data
    async def sendData(self, data):
        try:
            if self.client.is_connected and self.writer_characteristic is not None:
                await self.client.write_gatt_char(self.writer_characteristic.uuid, bytes(data))
        except Exception as ex:
            print(ex)

    # 读取寄存器 read register
    async def readReg(self, regAddr):
        # 封装读取指令并向串口发送数据 Encapsulate read instructions and send data to the serial port
        await self.sendData(self.get_readBytes(regAddr))

    # 写入寄存器 Write Register
    async def writeReg(self, regAddr, sValue):
        # 解锁 unlock
        await self.unlock()
        # 延迟100ms Delay 100ms
        time.sleep(0.1)
        # 封装写入指令并向串口发送数据
        await self.sendData(self.get_writeBytes(regAddr, sValue))
        # 延迟100ms Delay 100ms
        time.sleep(0.1)
        # 保存 save
        await self.save()

    # 读取指令封装 Read instruction encapsulation
    @staticmethod
    def get_readBytes(regAddr):
        # 初始化
        tempBytes = [None] * 5
        tempBytes[0] = 0xff
        tempBytes[1] = 0xaa
        tempBytes[2] = 0x27
        tempBytes[3] = regAddr
        tempBytes[4] = 0
        return tempBytes

    # 写入指令封装 Write instruction encapsulation
    @staticmethod
    def get_writeBytes(regAddr, rValue):
        # 初始化
        tempBytes = [None] * 5
        tempBytes[0] = 0xff
        tempBytes[1] = 0xaa
        tempBytes[2] = regAddr
        tempBytes[3] = rValue & 0xff
        tempBytes[4] = rValue >> 8
        return tempBytes

    # 解锁 unlock
    async def unlock(self):
        cmd = self.get_writeBytes(0x69, 0xb588)
        await self.sendData(cmd)

    # 保存 save
    async def save(self):
        cmd = self.get_writeBytes(0x00, 0x0000)
        await self.sendData(cmd)
