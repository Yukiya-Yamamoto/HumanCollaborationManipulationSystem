#!/usr/bin/env python3
# coding: UTF-8

#####################################################################################################################
#このノードは，上位アプリを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/06/28
#10/05 test_server.pyと単体検証を行うために，クラス部分をコメントアウト→統合試験でも成功
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

from abc import ABCMeta
from abc import abstractmethod
import rospy
from system_management.msg import *
from system_management.srv import *
from std_msgs.msg import Empty
import sys

class SystemManagementBase(metaclass=ABCMeta):
    """
    Base class that SystemManagement communication
    Define machine-dependent packages
    """
    @abstractmethod
    def execute(self, data):
        pass


class SystemManagementPublisher(SystemManagementBase):
    def __init__(self, topic_name, class_type):
        self.pub = rospy.Publisher(topic_name, class_type, queue_size = 10)

    def execute(self, data):
        self.pub.publish(data)

class SystemManagementClient(SystemManagementBase):
    def __init__(self, service_name, service_class):
        self.proxy = rospy.ServiceProxy(service_name, service_class)

    def execute(self, data):
        try :
            result = self.proxy(data)
            print(result)
            return result
        except rospy.ServiceException as e:
            rospy.loginfo("ServiceException : %s" % e)
            return False

#システム終了指令.
class SystemManagementHalt(SystemManagementPublisher):
    def __init__(self,):
        super().__init__('sys_manage_halt', Empty)

    def execute(self):
        super().execute(Empty())

#作業開始指令.
class SystemManagementTaskCommand(SystemManagementClient):
    def __init__(self):
        super().__init__('sys_manage_service', SystemManagement)

    def execute(self):
        print('{} start'.format(self.__class__.__name__))

        ###作業コマンド作成 ここから.

        #作業コマンドリスト作成.
        set_task_command_list = TaskCommandList()

        #ここまで.
        srv = SystemManagementRequest()
        srv.task_info_list = set_task_command_list
        ret = super().execute(srv)
        print('reslt ', ret.task_result_list.task_result)

def main(cmd:SystemManagementBase):
    print(cmd)
    if cmd is not None:
        rospy.init_node("system_management_node")
        cmd.execute()
    else:
        print('error ! not selected')

if __name__ == '__main__':
    args = sys.argv
    cmd = None
    if 1 == len(args):
        print("error command xxxx [yyyy]")
    elif 2 == len(args):
        #作業開始指令.
        if   'command' == args[1]:
            cmd = SystemManagementTaskCommand()
        #システム終了指令.
        elif 'halt' == args[1]:
            cmd = SystemManagementHalt()
    try:
        main(cmd)
    except rospy.ROSInterruptException:
        pass