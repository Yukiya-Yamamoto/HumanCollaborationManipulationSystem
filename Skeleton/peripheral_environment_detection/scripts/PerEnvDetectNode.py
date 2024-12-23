#!/usr/bin/env python3
# coding: UTF-8

#####################################################################################################################
#このノードは，周辺環境計測モジュールを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/06/28
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

#python,TFのライブラリを使用
import rospy
from rospy.topics import Publisher, Subscriber

from std_msgs.msg import *
from geometry_msgs.msg import *
from peripheral_environment_detection.msg import *
from peripheral_environment_detection.srv import * 

class AreaIntrusionDetect:
    def __init__(self):
        self.result_pub = Publisher('/intrusion_result', PerEnvDetectResult, queue_size=10)
        self.length_sub = Subscriber('/len_topic', Float32, self.UrgCallback)
        self.setup_req = rospy.Service('per_env_det_service', MonitoringAreaSetup, self.setup_request)
        self.setup_data = AreaSetupList()
        self.result_data = PerEnvDetectResult()
        self.urg_data = Float32()
        self.basis_length = 0.3 #[m]

        print("Initialization done")

    def setup_request(self, req_):
        srv = MonitoringAreaSetupResponse()
        self.setup_data = req_.setup_info_list
        if self.setup_data.area_setup_info:
            print("Setup succeeded")
            srv.setup_result.succeeded = 1
            srv.setup_result.fail = 0
            print("Setup Data")
            print(self.setup_data)
            print("AreaIntrusionDetection start!")
        else:
            print("Setup fail")
            srv.setup_result.succeeded = 0
            srv.setup_result.fail = 1
        return srv

    def UrgCallback(self, urg_):
        self.urg_data = urg_.data

    def mainloop(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.setup_data.area_setup_info:
                self.result_data.area_division.cooperative_work_range = self.setup_data.area_setup_info[0].area_division.cooperative_work_range
                self.result_data.area_division.non_cooperative_work_range = self.setup_data.area_setup_info[0].area_division.non_cooperative_work_range

                if (self.basis_length * self.setup_data.area_setup_info[0].area_division.non_cooperative_work_range > self.urg_data):
                    self.result_data.intrusion = True
                else:
                    self.result_data.intrusion = False

                self.result_pub.publish(self.result_data)
                #TODO サブスクライバー作成.

            r.sleep()

if __name__ == "__main__":
    rospy.init_node("peripheral_environment_detection_node")
    aid = AreaIntrusionDetect()
    aid.mainloop()
    
