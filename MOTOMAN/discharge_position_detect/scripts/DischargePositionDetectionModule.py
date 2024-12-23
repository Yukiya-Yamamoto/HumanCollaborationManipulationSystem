#!/usr/bin/env python3
# coding: UTF-8

#####################################################################################################################
#このノードは，排出位置検出サブモジュールを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/08/28
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

#python,TFのライブラリを使用
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from discharge_position_detect.srv import *

def response_data(req):
    print(req)
    srv = DischargePositionDetectionResultResponse()
    srv.task_command_id = 1

    #リストを受信する.

    #排出位置候補から排出場所決める.

    print(srv)
    return srv

def dis_pos_detect_server():
    rospy.init_node('discharge_position_detect_server')
    rospy.Service('discharge_position_detect_server', DischargePositionDetectionResult, response_data)
    rospy.spin()

if __name__ == "__main__":
   dis_pos_detect_server()