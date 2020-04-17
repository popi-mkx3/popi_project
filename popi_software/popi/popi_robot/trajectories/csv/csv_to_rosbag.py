#!/usr/bin/env python
import rospy
import rosbag
import csv
import sys
from std_msgs.msg import Float64

with rosbag.Bag(sys.argv[1] + '.bag', 'w') as bag:
    with open(sys.argv[1] + '.csv') as csvfile:
         reader = csv.DictReader(csvfile)
         for row in reader:
             timestamp = rospy.Time.from_sec(float(row['%time']))
             aileAVD_msg = Float64(float(row['aileAVD']))
             aileAVG_msg = Float64(float(row['aileAVG']))
             aileARD_msg = Float64(float(row['aileARD']))
             aileARG_msg = Float64(float(row['aileARG']))
             epauleAVD_msg = Float64(float(row['epauleAVD']))
             epauleAVG_msg = Float64(float(row['epauleAVG']))
             epauleARD_msg = Float64(float(row['epauleARD']))
             epauleARG_msg = Float64(float(row['epauleARG']))
             coudeAVD_msg = Float64(float(row['coudeAVD']))
             coudeAVG_msg = Float64(float(row['coudeAVG']))
             coudeARD_msg = Float64(float(row['coudeARD']))
             coudeARG_msg = Float64(float(row['coudeARG']))
         
             bag.write("/popi/aileAVD_eff_position_controller/command", aileAVD_msg, timestamp)
             bag.write("/popi/aileAVG_eff_position_controller/command", aileAVG_msg, timestamp)
             bag.write("/popi/aileARD_eff_position_controller/command", aileARD_msg, timestamp)
             bag.write("/popi/aileARG_eff_position_controller/command", aileARG_msg, timestamp)
             bag.write("/popi/epauleAVD_eff_position_controller/command", epauleAVD_msg, timestamp)
             bag.write("/popi/epauleAVG_eff_position_controller/command", epauleAVG_msg, timestamp)
             bag.write("/popi/epauleARD_eff_position_controller/command", epauleARD_msg, timestamp)
             bag.write("/popi/epauleARG_eff_position_controller/command", epauleARG_msg, timestamp)
             bag.write("/popi/coudeAVD_eff_position_controller/command", coudeAVD_msg, timestamp)
             bag.write("/popi/coudeAVG_eff_position_controller/command", coudeAVG_msg, timestamp)
             bag.write("/popi/coudeARD_eff_position_controller/command", coudeARD_msg, timestamp)
             bag.write("/popi/coudeARG_eff_position_controller/command", coudeARG_msg, timestamp)


