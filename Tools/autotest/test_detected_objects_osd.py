#!/usr/bin/env python3
"""
Test script for OSD detected objects feature.
Sends custom MAVLink messages (ID 50100) with bounding box coordinates
to draw bounding boxes on the OSD.

Requires: pymavlink
Usage: python test_detected_objects_osd.py --connect udp:127.0.0.1:14550
"""

import struct
import socket
import argparse
import time
import sys
import math

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink required: pip install pymavlink")
    sys.exit(1)

DETECTED_OBJECT_MSG_ID = 50100


def pack_detected_object(tracker_id, count, x1, y1, x2, y2, confidence):
    payload = struct.pack(
        '<BBHBffffB',
        0,           # target_system
        0,           # target_component
        tracker_id,  # tracker_id (uint16)
        count,       # count (uint8)
        x1, y1,      # top-left (float, normalized 0-1)
        x2, y2,      # bottom-right (float, normalized 0-1)
        confidence,  # confidence (uint8, 0-100)
    )
    return payload


def send_detected_object(msg_dst, tracker_id, count, x1, y1, x2, y2, confidence=100):
    payload = pack_detected_object(tracker_id, count, x1, y1, x2, y2, confidence)
    msg_dst.mav.send(
        mavutil.mavlink.MAVLink_message(
            DETECTED_OBJECT_MSG_ID,
            payload,
        )
    )


def send_raw_mavlink(conn, target_sys, target_comp, tracker_id, count,
                     x1, y1, x2, y2, confidence=100):
    payload = pack_detected_object(tracker_id, count, x1, y1, x2, y2, confidence)

    header = bytearray()
    header.append(0xFE)  # MAVLink v1 start byte
    payload_len = len(payload)
    header.append(payload_len)
    header.append(0)  # incompat flags
    header.append(0)  # compat flags
    header.append(target_sys & 0xFF)
    header.append(target_comp & 0xFF)
    msg_id = DETECTED_OBJECT_MSG_ID
    header.extend(struct.pack('<I', msg_id))
    header.extend(payload)

    from pymavlink import mavutil
    import array
    crc_extra = 0  # custom message, no CRC extra
    crc = mavutil.x25crc(array.array('B', header[1:] + payload))
    header.extend(struct.pack('<H', crc.crc))

    if hasattr(conn, 'write'):
        conn.write(bytes(header))
    else:
        conn.send(bytes(header))


def test_static_boxes(conn, sysid):
    print("Test 1: Static bounding boxes")

    boxes = [
        (0.1, 0.2, 0.3, 0.5),
        (0.6, 0.1, 0.9, 0.4),
        (0.2, 0.6, 0.5, 0.9),
        (0.7, 0.5, 0.95, 0.85),
    ]

    for i, (x1, y1, x2, y2) in enumerate(boxes):
        send_raw_mavlink(conn, sysid, 0, i, len(boxes), x1, y1, x2, y2)
        print(f"  Sent object {i}: ({x1:.1f},{y1:.1f})-({x2:.1f},{y2:.1f})")
        time.sleep(0.05)

    print("  Waiting 5s for OSD display...")
    time.sleep(5)


def test_moving_box(conn, sysid):
    print("Test 2: Moving bounding box")

    cx, cy = 0.5, 0.5
    w, h = 0.15, 0.2

    for step in range(60):
        angle = step * 6 * math.pi / 180
        dx = 0.2 * math.sin(angle)
        dy = 0.15 * math.cos(angle)

        x1 = max(0.0, min(1.0, cx + dx - w / 2))
        y1 = max(0.0, min(1.0, cy + dy - h / 2))
        x2 = max(0.0, min(1.0, cx + dx + w / 2))
        y2 = max(0.0, min(1.0, cy + dy + h / 2))

        send_raw_mavlink(conn, sysid, 0, 42, 1, x1, y1, x2, y2)
        time.sleep(0.1)

    print("  Done")


def test_expiry(conn, sysid):
    print("Test 3: Object expiry (should disappear after 500ms)")

    send_raw_mavlink(conn, sysid, 0, 0, 1, 0.3, 0.3, 0.7, 0.7)
    print("  Sent object, waiting 2s...")
    time.sleep(2)
    print("  Object should have disappeared from OSD")


def test_max_objects(conn, sysid):
    print("Test 4: Max objects (8+) - oldest should be replaced")

    for i in range(10):
        x1 = 0.05 + i * 0.09
        x2 = x1 + 0.08
        send_raw_mavlink(conn, sysid, 0, i + 100, 10, x1, 0.3, x2, 0.7)
        time.sleep(0.05)

    print("  Sent 10 objects with different tracker IDs, should see last 8")
    time.sleep(3)


def main():
    parser = argparse.ArgumentParser(description='Test OSD detected objects')
    parser.add_argument('--connect', default='udp:127.0.0.1:14550',
                        help='MAVLink connection string')
    parser.add_argument('--test', choices=['static', 'moving', 'expiry',
                                           'max', 'all'],
                        default='all', help='Which test to run')
    args = parser.parse_args()

    print(f"Connecting to {args.connect}...")
    conn = mavutil.mavlink_connection(args.connect)

    conn.wait_heartbeat()
    sysid = conn.target_system
    print(f"Connected to system {sysid}")

    if args.test in ('static', 'all'):
        test_static_boxes(conn, sysid)
    if args.test in ('moving', 'all'):
        test_moving_box(conn, sysid)
    if args.test in ('expiry', 'all'):
        test_expiry(conn, sysid)
    if args.test in ('max', 'all'):
        test_max_objects(conn, sysid)

    print("Done")


if __name__ == '__main__':
    main()
