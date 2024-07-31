import serial
import time
import csv
import json
import rospy
from std_msgs.msg import String


# Initialize the node
rospy.init_node('localization_node')

# Configure the serial connection
DWM = serial.Serial(port="/dev/ttyACM1", baudrate=115200)
print("Connected to " + DWM.name)
DWM.write("\r\r".encode())
print("Encode")
time.sleep(1)
DWM.write("lec\r".encode())
print("Encode")
time.sleep(1)


# Open a CSV file to save data
# csv_file = open('rapid_localization_data.csv', mode='w', newline='')
# fieldnames = ['Time', 'Type', 'Anchor_Count', 'Anchor_Data', 'Tag_Position']
# writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
# writer.writeheader()

# Create a publisher for the localization data
localization_pub = rospy.Publisher('localization_data', String, queue_size=10)
def shutdown():
    # Close the serial connection
    DWM.write("\r".encode())
    DWM.close()

    # print("Serial connection closed")
    # Close the CSV file
    # csv_file.close()
    # print("CSV file closed")
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

def main():
    rospy.on_shutdown(shutdown)
    while not rospy.is_shutdown():
        data = DWM.readline()
        if data:
            row=parse_data(data)
            # Publish the data
            localization_pub.publish(json.dumps(row))

            # For saving to a csv file
            # writer.writerow(row)

if __name__ == "__main__":
    main()