import serial
import time
import csv
import json
import rospy
from std_msgs.msg import String


# Initialize the node
rospy.init_node('localization_node')

# Create a publisher for the localization data
localization_pub = rospy.Publisher('localization_data', String, queue_size=10)

def connect_serial():
    try:
        DWM = serial.Serial(port="/dev/ttyACM1", baudrate=115200)
        rospy.loginfo("Connected to " + DWM.name)
        DWM.write("\r\r".encode())
        print("Encode 1")
        time.sleep(1)
        DWM.write("lec\r".encode())
        print("Encode 2")
        time.sleep(1)
    except serial.SerialException as e:
        rospy.logerr(f"Failed to connect to serial port: {e}")
        DWM = None
    return DWM

# Configure the serial connection
DWM = connect_serial()

def shutdown():
    # Close the serial connection
    DWM.write("\r".encode())
    DWM.close()
    rospy.loginfo("Serial connection closed")

def parse_data(data):
    data = data.decode().strip().split(",")
    # print(data)
    if "DIST" in data:
        time_stamp = rospy.Time.now()
        anchor_number = int(data[1])
        anchor_data = []
        tag_position = []
        for i in range(anchor_number):
            start_idx = 2 + i*6
            anchor_info = {
                'Anchor': data[start_idx],
                'ID': data[start_idx+1],
                'X': data[start_idx+2],
                'Y': data[start_idx+3],
                'Z': data[start_idx+4],
                'Dist': data[start_idx+5]
            }
            anchor_data.append(anchor_info)
        if "POS" in data:
            pos_index = data.index("POS")
            tag_position = {
                "x": data[pos_index+1],
                "y": data[pos_index+2],
                "z": data[pos_index+3]
            }
            row = {
                'Time': time_stamp,
                'Type': 'DIST',
                'Anchor_Count': anchor_number,
                'Anchor_Data': json.dumps(anchor_data),
                'Tag_Position': json.dumps(tag_position)
            }
            localization_pub.publish(json.dumps(row))
    return

def main():
    rospy.on_shutdown(shutdown)
    while not rospy.is_shutdown():
        if DWM is None or not DWM.is_open:
            DWM = connect_serial()
            if DWM is None:
                rospy.logerr("Reconnection failed, retrying...")
                time.sleep(1)
                continue
        try:
            data = DWM.readline()
            if data:
                parse_data(data)
        except serial.SerialException as e:
            rospy.logerr(f"Serial Exception: {e}")
            DWM.close()
            DWM = None
        except Exception as e:
            rospy.logerr(f"Unexpected Exception: {e}")

if __name__ == "__main__":
    main()
