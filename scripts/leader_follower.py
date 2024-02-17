#!/usr/bin/env python
from dynamixel_sdk import *
import numpy as np
import time

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_GOAL_CURRENT = 102
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
PROTOCOL_VERSION = 2.0
BAUDRATE = 3000000
# LEADER_DEVICENAME = '/dev/ttyUSB0'
# FOLLOWER_DEVICENAME = '/dev/ttyUSB1'
LEADER_DEVICENAME = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUD0-if00-port0'
FOLLOWER_DEVICENAME = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUU6-if00-port0'
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
ADDR_OPERATING_MODE = 11
CURRENT_BASE_POSITION_CONTROL_MODE = 5

class DynamixelPort:
    method_dict = {
        1 : "write1ByteTxRx",
        2 : "write2ByteTxRx",
        4 : "write4ByteTxRx"
    }

    def __init__(self, device, dxl_ids):
        self.device = device
        self.dxl_ids = dxl_ids
        self.portHandler = PortHandler(device)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()
        self.setup()
        self.pos_writer = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, 4)
        self.cur_writer = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_CURRENT, 2)
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_CURRENT, 10)
        for dxl_id in dxl_ids:
            dxl_addparam_result = self.groupSyncRead.addParam(dxl_id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % dxl_id)
                quit()
        self.present_currents = np.zeros((8), np.int16)
        self.present_positions = np.zeros((8), np.int32)

    def writeTxRx(self, dxl_id, addr, value):
        dxl_comm_result, dxl_error = self.packetHandler.__getattribute__(self.method_dict[value.itemsize])(self.portHandler, dxl_id, addr, int(value))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            exit()
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            exit()

    def setup(self):
        for dxl_id in self.dxl_ids:
            self.writeTxRx(dxl_id, ADDR_TORQUE_ENABLE, np.int8(TORQUE_DISABLE))
            self.writeTxRx(dxl_id, ADDR_OPERATING_MODE, np.int8(CURRENT_BASE_POSITION_CONTROL_MODE))
            self.writeTxRx(dxl_id, ADDR_TORQUE_ENABLE, np.int8(TORQUE_ENABLE))

    def cleanup(self):
        for dxl_id in self.dxl_ids:
            self.writeTxRx(dxl_id, ADDR_TORQUE_ENABLE, np.int8(TORQUE_DISABLE))
        self.portHandler.closePort()

    def fetch_present_status(self):
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            return
        for i, dxl_id in enumerate(self.dxl_ids):
            if self.groupSyncRead.isAvailable(dxl_id, ADDR_PRESENT_CURRENT, 10):
                v = self.groupSyncRead.getData(dxl_id, ADDR_PRESENT_CURRENT, 2)
                self.present_currents[i] = ((0x8000 + v) % 0x10000) - 0x8000
                v = self.groupSyncRead.getData(dxl_id, ADDR_PRESENT_POSITION, 4)
                self.present_positions[i] = ((0x80000000 + v) % 0x100000000) - 0x80000000

    def set_goal_positions(self, pos, cur):
        for dxl_id, p, c in zip(self.dxl_ids, pos, cur):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(p)), DXL_HIBYTE(DXL_LOWORD(p)), DXL_LOBYTE(DXL_HIWORD(p)), DXL_HIBYTE(DXL_HIWORD(p))]
            param_goal_current = [DXL_LOBYTE(DXL_LOWORD(c)), DXL_HIBYTE(DXL_LOWORD(c))]
            if self.pos_writer.addParam(dxl_id, param_goal_position) != True:
                print("[ID:%03d] pos_writer addparam failed" % dxl_id)
            if self.cur_writer.addParam(dxl_id, param_goal_current) != True:
                print("[ID:%03d] cur_writer addparam failed" % dxl_id)
        dxl_comm_result = self.pos_writer.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        self.pos_writer.clearParam()
        dxl_comm_result = self.cur_writer.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        self.cur_writer.clearParam()

leader = DynamixelPort(LEADER_DEVICENAME, list(range(1,9)))
follower = DynamixelPort(FOLLOWER_DEVICENAME, list(range(1,9)))

lbase = np.array([ -1000,  3000,  0,  1400,  -2000, 800,  1500,  0], dtype=np.int32)
lrange = np.array([4000, 2000, 4000, 1800, 4000, 2400, 4000, 500], dtype=np.int32)
fbase = np.array([262, 1024, 262, 228, 262, 1024, 148, 1991], dtype=np.int32)
fend = np.array([3834, 3072, 3834, 2048, 3834, 3072, 3948, 3072], dtype=np.int32)
frange = fend - fbase
def leader2follower():
    lnorm = np.clip((leader.present_positions - lbase) / lrange, 0.01, 0.99)
    pos = leader.present_positions
    cur = np.array([500, 500, 500, 500, 500, 500, 500, 500], dtype=np.int16)
    return pos, cur, lnorm

leader.set_goal_positions([3600, 2800, 2600, 500, 2600, 3500, 1500, 1700],
                          [50, 0, 50, 0, 50, 0, 40, 30])

i = 0
running = True
while running:
    try:
        leader.fetch_present_status()
        pos, cur, lnorm = leader2follower()
        follower.set_goal_positions(pos, cur)
        follower.fetch_present_status()
        pos = follower.present_positions
        cur = follower.present_currents
        fnorm = (pos - fbase) / frange
        i += 1
        if i % 100 == 0:
            for id, l, f, c in zip(range(1,9), leader.present_positions, follower.present_positions, follower.present_currents):
                print("{:02d} {:4d} {:4d} {:4d}".format(id, l, f, c))
            print("----------------")
    except KeyboardInterrupt:
        print("key pressed")
        running = False

follower.cleanup()
leader.cleanup()
