import argparse
import logging
import os
import sys
import time
from pathlib import Path

import canopen

DEFAULT_SERIAL_PORT = "/dev/cu.usbmodem101"
DEFAULT_CHANNEL = DEFAULT_SERIAL_PORT
DEFAULT_BITRATE = 1_000_000
DEFAULT_NODE_ID = 0x7C
DEFAULT_BIN_PATH = Path("build-sysbuild/template/zephyr/zephyr.signed.bin")

DEFAULT_BLOCK_TRANSFER = False
DEFAULT_DOWNLOAD_BUFFER_SIZE = 889
DEFAULT_STATUS_TIMEOUT_S = 30.0
DEFAULT_BOOTUP_TIMEOUT_S = 20.0
DEFAULT_SDO_TIMEOUT_S = 3.0
DEFAULT_SDO_RETRIES = 3
DEFAULT_CONFIRM_IMAGE = False

DEFAULT_REQUEST_CRC = False
DEFAULT_THROTTLE_DELAY_S = 0.0001

H1F50_PROGRAM_DATA = 0x1F50
H1F51_PROGRAM_CTRL = 0x1F51
H1F56_PROGRAM_SWID = 0x1F56
H1F57_FLASH_STATUS = 0x1F57

PROGRAM_CTRL_STOP = 0x00
PROGRAM_CTRL_START = 0x01
PROGRAM_CTRL_CLEAR = 0x03
PROGRAM_CTRL_ZEPHYR_CONFIRM = 0x80


def parse_args():
    p = argparse.ArgumentParser()

    p.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    p.add_argument("--channel", default=None)
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE)
    p.add_argument("--node-id", type=parse_int, default=DEFAULT_NODE_ID)
    p.add_argument("--bin", dest="bin_path", type=Path, default=DEFAULT_BIN_PATH)

    p.add_argument("--block-transfer", action="store_true", default=DEFAULT_BLOCK_TRANSFER)
    p.add_argument("--download-buffer-size", type=int, default=DEFAULT_DOWNLOAD_BUFFER_SIZE)
    p.add_argument("--status-timeout", type=float, default=DEFAULT_STATUS_TIMEOUT_S)
    p.add_argument("--bootup-timeout", type=float, default=DEFAULT_BOOTUP_TIMEOUT_S)
    p.add_argument("--sdo-timeout", type=float, default=DEFAULT_SDO_TIMEOUT_S)
    p.add_argument("--sdo-retries", type=int, default=DEFAULT_SDO_RETRIES)

    p.add_argument("--confirm", dest="confirm_image", action="store_true", default=DEFAULT_CONFIRM_IMAGE)
    p.add_argument("--no-confirm", dest="confirm_image", action="store_false")

    p.add_argument("--request-crc", action="store_true", default=DEFAULT_REQUEST_CRC)

    p.add_argument("--throttle-delay", type=float, default=DEFAULT_THROTTLE_DELAY_S)

    p.add_argument("--debug", action="store_true", help="Enable verbose CAN logging")

    return p.parse_args()


def parse_int(s):
    return int(s, 0)


def create_object_dictionary():
    objdict = canopen.objectdictionary.ObjectDictionary()

    arr = canopen.objectdictionary.Array("Program data", H1F50_PROGRAM_DATA)
    var = canopen.objectdictionary.Variable("", H1F50_PROGRAM_DATA, subindex=1)
    var.data_type = canopen.objectdictionary.DOMAIN
    arr.add_member(var)
    objdict.add_object(arr)

    arr = canopen.objectdictionary.Array("Program control", H1F51_PROGRAM_CTRL)
    var = canopen.objectdictionary.Variable("", H1F51_PROGRAM_CTRL, subindex=1)
    var.data_type = canopen.objectdictionary.UNSIGNED8
    arr.add_member(var)
    objdict.add_object(arr)

    arr = canopen.objectdictionary.Array("Program software ID", H1F56_PROGRAM_SWID)
    var = canopen.objectdictionary.Variable("", H1F56_PROGRAM_SWID, subindex=1)
    var.data_type = canopen.objectdictionary.UNSIGNED32
    arr.add_member(var)
    objdict.add_object(arr)

    arr = canopen.objectdictionary.Array("Flash status", H1F57_FLASH_STATUS)
    var = canopen.objectdictionary.Variable("", H1F57_FLASH_STATUS, subindex=1)
    var.data_type = canopen.objectdictionary.UNSIGNED32
    arr.add_member(var)
    objdict.add_object(arr)

    return objdict


def wait_flash_status_ok(flash_sdo, timeout_s):
    end = time.time() + timeout_s
    status = int(flash_sdo.raw)

    while status != 0 and time.time() < end:
        time.sleep(0.1)
        status = int(flash_sdo.raw)

    return status


def main():
    args = parse_args()

    if args.debug:
        logging.basicConfig(level=logging.DEBUG, format='%(asctime)s [%(levelname)s] %(name)s: %(message)s')
        logging.getLogger('canopen').setLevel(logging.DEBUG)
        pass
    else:
        logging.basicConfig(level=logging.INFO, format='%(message)s')

    channel = args.channel or args.serial_port
    bin_path = args.bin_path

    if not bin_path.is_file():
        logging.error(f"Binary not found: {bin_path}")
        return 1

    size = os.path.getsize(bin_path)

    network = canopen.Network()
    network.connect(interface="slcan", channel=channel, bitrate=args.bitrate)

    if args.block_transfer:
        original_send = network.bus.send

        def throttled_send(msg, timeout=None):
            original_send(msg, timeout)
            time.sleep(args.throttle_delay)

        network.bus.send = throttled_send

    node = network.add_node(args.node_id, create_object_dictionary())
    node.sdo.MAX_RETRIES = args.sdo_retries
    node.sdo.RESPONSE_TIMEOUT = args.sdo_timeout

    data_sdo = node.sdo[H1F50_PROGRAM_DATA][1]
    ctrl_sdo = node.sdo[H1F51_PROGRAM_CTRL][1]
    swid_sdo = node.sdo[H1F56_PROGRAM_SWID][1]
    flash_sdo = node.sdo[H1F57_FLASH_STATUS][1]

    logging.info(f"Software ID: 0x{int(swid_sdo.raw):08X}")

    node.nmt.state = "PRE-OPERATIONAL"
    time.sleep(0.5)

    ctrl_sdo.raw = PROGRAM_CTRL_STOP
    ctrl_sdo.raw = PROGRAM_CTRL_CLEAR

    status = wait_flash_status_ok(flash_sdo, args.status_timeout)
    if status != 0:
        logging.error(f"CLEAR failed, flash status=0x{status:08X}")
        network.disconnect()
        return 2

    with open(bin_path, "rb") as infile:
        outfile = data_sdo.open(
            "wb",
            buffering=args.download_buffer_size,
            size=size,
            block_transfer=args.block_transfer,
            request_crc_support=args.request_crc
        )
        outfile.write(infile.read())
        outfile.close()

    status = wait_flash_status_ok(flash_sdo, args.status_timeout)
    if status != 0:
        logging.error(f"Download failed, flash status=0x{status:08X}")
        network.disconnect()
        return 3

    logging.info(f"Software ID after download: 0x{int(swid_sdo.raw):08X}")

    ctrl_sdo.raw = PROGRAM_CTRL_START
    node.nmt.wait_for_bootup(timeout=args.bootup_timeout)

    logging.info(f"Software ID after reboot: 0x{int(swid_sdo.raw):08X}")

    if args.confirm_image:
        node.nmt.state = "PRE-OPERATIONAL"
        time.sleep(0.5)
        ctrl_sdo.raw = PROGRAM_CTRL_ZEPHYR_CONFIRM

    network.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
