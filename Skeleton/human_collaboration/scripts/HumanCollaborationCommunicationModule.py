#!/usr/bin/env python3
# coding: UTF-8

from abc import ABCMeta
from abc import abstractmethod
import rospy, tf2_ros
from peripheral_environment_detection.msg import *
from peripheral_environment_detection.srv import *
from work_detection.msg import *
from work_detection.srv import *
from system_management.msg import *
from system_management.srv import * 
from discharge_position_detect.msg import *
from discharge_position_detect.srv import *
from HumanCollaborationEventModule import HumanCollaborationEventPublisher, HumanCollaborationTaskCommand, HumanCollaborationWorkPresenceArea
from HumanCollaborationEventModule import HumanCollaborationCandidateDischargeLocationAreaList, HumanCollaborationCandidateDischargeLocationArea, HumanCollaborationTaskCommandList
from HumanCollaborationToolModule import HumanCollaborationTool
from EnumerateModule import EnumEvent
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Point
import threading

#協働通信クラスの抽象クラス.
class HumanCollaborationCommunication(metaclass=ABCMeta):
    """
    Base class that HumanCollaboration communication
    Define machine-dependent packages
    """
    @abstractmethod
    def execute(self, object):
        pass

#協働パブリッシャークラスの抽象クラス.
class HumanCollaborationPublisher(HumanCollaborationCommunication):
    """
    Base class that HumanCollaboration communication for Publisher
    Define machine-dependent packages
    """
    def __init__(self, topic_name, datatype):
        self.pub = rospy.Publisher(topic_name, datatype, queue_size=10)

    def execute(self, object):
        pass

#協働サブスクライバークラスの抽象クラス.
class HumanCollaborationSubscriver(HumanCollaborationCommunication):
    """
    Base class that HumanCollaboration communication for Subscriver
    Define machine-dependent packages
    """
    def __init__(self, topic_name, datatype, callback):
        self.sub = rospy.Subscriber(topic_name, datatype, callback)
    def execute(self, object):
        pass

#協働クライアントクラスの抽象クラス.
class HumanCollaborationClient(HumanCollaborationCommunication):
    """
    Base class that HumanCollaboration communication for request
    Define machine-dependent packages
    """
    def __init__(self, service_name, service_class):
        self.proxy = rospy.ServiceProxy(service_name, service_class)

    def execute(self, object):
        pass

#協働サーバークラスの抽象クラス.
class HumanCollaborationServer(HumanCollaborationCommunication):
    """
    Base class that HumanCollaboration communication for response
    Define machine-dependent packages
    """
    def __init__(self, service_name, service_class, callback_impl):
        self.server = rospy.Service(service_name, service_class, callback_impl)

    def service_delete(self, msg=''):
        self.server.shutdown(msg)

    def execute(self, object):
        pass

    def wait(self, time, condition):
        self.condition = condition
        while self.condition:
            rospy.sleep(time)

    def set_wait_condition(self, condition):
        self.condition = condition

#システム終了コマンド受信クラス.
class SystemHalt(HumanCollaborationSubscriver):
    def __init__(self):
        super().__init__("sys_manage_halt", Empty, self.receive_system_halt)
        HumanCollaborationTool.loginfo('{} start '.format(self.__class__.__name__))

    #システム終了コマンド受信.
    def receive_system_halt(self, data):
        HumanCollaborationTool.loginfo('receive_system_halt {}'.format(data))
        HumanCollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_end()), data)

    def execute(self, object):
        pass

#システム中断コマンド受信クラス.
class SystemPause(HumanCollaborationSubscriver):
    def __init__(self):
        super().__init__("sys_manage_pause", Bool, self.receive_system_pause)
        HumanCollaborationTool.loginfo('{} start '.format(self.__class__.__name__))
    
    #システム中断コマンド受信.
    def receive_system_pause(self, data):
        if data.data:
            HumanCollaborationTool.loginfo('receive_system_pause {}'.format(data))
            HumanCollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_pause()), data.data)
        else:
            HumanCollaborationTool.loginfo('receive_system_pause {}'.format(data))
            HumanCollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_pausecancel()), data.data)

    def execute(self, object):
        pass

#協働作業コマンド受信クラス.
class SystemCollabo(HumanCollaborationSubscriver):
    def __init__(self):
        super().__init__("sys_manage_collabo", Bool, self.receive_system_collabo)
        HumanCollaborationTool.loginfo('{} start'.format(self.__class__.__name__))

    #協働作業コマンド受信.
    def receive_system_collabo(self, data):
        if data.data:
            HumanCollaborationTool.loginfo('receive_system_collabo {}'.format(data))
            HumanCollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_collabo()), data.data)
        else:
            HumanCollaborationTool.loginfo('receive_system_collabo_end {}'.format(data))
            HumanCollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_collaboend()), data.data)

    def execute(self, object):
        pass

#協働サブスクライバースレッド.
class HumanCollaborationSubscriverThread(threading.Thread):
    def __init__(self):
        super(HumanCollaborationSubscriverThread, self).__init__()
        self.syspause = SystemPause()
        self.syscollabo = SystemCollabo()
        self.syshalt = SystemHalt()

    def run(self):
        rospy.spin()

#作業開始コマンド受信クラス.
class TaskCommandServer(HumanCollaborationServer):
    def __init__(self):
        super().__init__('sys_manage_service', SystemManagement, self.recv_task_command)
        self.rsp = None
        self.is_rsp = False

    def service_delete(self):
        super().service_delete('sys_manage_servece deleted')

    #作業開始コマンド受信.
    def recv_task_command(self, data):
        HumanCollaborationTool.loginfo("TaskCommandServer::recv_task_command")

        #受信データ変換処理.
        command_list = HumanCollaborationTaskCommandList()
        for exchange_task in data.task_info_list.task_command_list :
            command = HumanCollaborationTaskCommand()
            command.set_member(exchange_task.task_command_id, exchange_task.work_type_id, exchange_task.number_of_items_picked)
            work_presence = HumanCollaborationWorkPresenceArea()
            work_presence.set_member(exchange_task.work_presence_area.start_point.x,
                                     exchange_task.work_presence_area.start_point.y,
                                     exchange_task.work_presence_area.start_point.z,
                                     exchange_task.work_presence_area.end_point.x,
                                     exchange_task.work_presence_area.end_point.y,
                                     exchange_task.work_presence_area.end_point.z)
            command.set_WorkPresenceArea(work_presence)
            discharge_list = HumanCollaborationCandidateDischargeLocationAreaList()

            for rcv_dis_area in exchange_task.candidate_discharge_location.candidate_discharge_location_area_list:
                dis_area = HumanCollaborationCandidateDischargeLocationArea()
                dis_area.set_member(rcv_dis_area.start_point.x,
                                    rcv_dis_area.start_point.y,
                                    rcv_dis_area.start_point.z,
                                    rcv_dis_area.end_point.x,
                                    rcv_dis_area.end_point.y,
                                    rcv_dis_area.end_point.z)
                discharge_list.add(dis_area)

            command.set_CandidateDischargeLocationAreaList(discharge_list)
            command_list.add(command)

        #受信内容をオブザーバ経由で通知する.
        HumanCollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_workstart()), command_list)
        self.wait(0.1, self.is_rsp is False)
        self.is_rsp = False
        res = TaskResult()
        res.task_command_id = self.task_command_id
        res.number_of_items_picked = self.number_of_items_picked
        res.task_result = self.task_result
        self.rsp = res
        return self.rsp

    #作業完了時に返信オブジェクトを記憶し、処理完了とする.
    def execute(self, object, task_command_id, number_of_items_picked, task_result):
        self.rsp = object
        self.is_rsp = True
        self.task_command_id = task_command_id
        self.number_of_items_picked = number_of_items_picked
        self.task_result = task_result
        self.set_wait_condition(self.is_rsp is False)
        return True

#一時停止コマンド受信クラス.
class TaskSuspendServer(HumanCollaborationServer):
    def __init__(self):
        super().__init__('sys_manage_service_suspend', SystemManagementSuspend, self.recv_task_suspend)
        self.task_comand_id = 0
        self.rsp = None
        self.is_rsp = False

    def service_delete(self):
        super().service_delete('sys_manage_service_suspend deleted')

    #一時停止コマンド受信.
    def recv_task_suspend(self, data):
        HumanCollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_worksuspend()), data.task_command_id)
        self.task_comand_id = data.task_command_id
        self.wait(0.1, self.is_rsp is False)
        self.is_rsp = False
        return self.rsp

    def execute(self, object):
        rsp = SystemManagementSuspendResponse()
        rsp.task_result_list.task_command_id = object.task_command_id
        rsp.task_result_list.number_of_items_picked = object.number_of_items_picked
        rsp.task_result_list.task_result = True
        self.rsp = object
        self.is_rsp = True
        self.set_wait_condition(self.is_rsp is False)
        return True

#作業終了コマンド受信クラス.
class TaskFinalServer(HumanCollaborationServer):
    def __init__(self):
        super().__init__('sys_manage_service_fin', SystemManagementFinal, self.recv_task_final)
        self.rsp = None
        self.is_rsp = False

    def service_delete(self):
        super().service_delete('sys_manage_service_fin deleted')

    #作業終了コマンド.
    def recv_task_final(self, data):
        HumanCollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_workend()), data.task_command_final)
        self.wait(0.1, self.is_rsp is False)
        self.is_rsp = False
        return self.rsp
    
    def execute(self, object):
        rsp = SystemManagementFinalResponse()
        rsp.task_result_list.task_command_id = object.task_command_id
        rsp.task_result_list.number_of_items_picked = object.number_of_items_picked
        rsp.task_result_list.task_result = True
        self.rsp = rsp
        self.is_rsp = True
        self.set_wait_condition(self.is_rsp is False)
        return True

#状態確認コマンド受信クラス.
class TaskGetStateServer(HumanCollaborationServer):
    def __init__(self):
        super().__init__('sys_manage_service_get_state', SystemManagementState, self.recv_get_state)
        self.rsp = None
        self.is_rsp = False

    def service_delete(self):
        super().service_delete('sys_manage_service_get_state deleted')

    #状態確認コマンド受信.
    def recv_get_state(self, data):
        HumanCollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_getstate()), data.req)
        self.wait(0.1, self.is_rsp is False)
        rsp = SystemManagementStateResponse()
        rsp.res.state = self.rsp
        self.is_rsp = False
        return rsp
    
    def execute(self, object):
        self.rsp = object
        self.is_rsp = True
        self.set_wait_condition(self.is_rsp is False)
        return True

#サーバーにワーク認識指令を出すクラス.
class WorkDetectionClient(HumanCollaborationClient):
    def __init__(self):
        super().__init__('work_det_service', WorkDetection)
        self.work_detect_result = WorkDetectionResultList()

    def set_presence_area(self, req_task_id, req_work_id, sx, sy, sz, ex, ey, ez):
        self.req_task_id = req_task_id
        self.req_work_id = req_work_id
        set_area = TargetArea()
        set_area.start_point.x = sx
        set_area.start_point.y = sy
        set_area.start_point.z = sz
        set_area.end_point.x = ex
        set_area.end_point.y = ey
        set_area.end_point.z = ez
        self.req_target_area = set_area

    def execute(self, object):
        try:
            result = self.proxy(self.req_task_id, self.req_work_id, self.req_target_area)
            HumanCollaborationTool.loginfo("Detection Result")
            HumanCollaborationTool.loginfo(result.work_detection_result_list)
            self.work_detect_result = result.work_detection_result_list
            return True
        except HumanCollaborationTool.ServiceException as e:
            HumanCollaborationTool.loginfo("ServiceException : %s" % e)
            return False

#サーバーに排出場所検出指令を出すクラス.
class DischargePositionClient(HumanCollaborationClient):
    def __init__(self):
        super().__init__('discharge_position_detect_server', DischargePositionDetectionResult)
        self.taskid = 0
    
    def set_discharge_area(self, sx, sy, sz, ex, ey, ez):
        discharge_position = DischargeArea()
        discharge_position.start_point.x = sx
        discharge_position.start_point.y = sy
        discharge_position.start_point.z = sz
        discharge_position.end_point.x = ex
        discharge_position.end_point.y = ey
        discharge_position.end_point.z = ez
        self.req_discharge_position = discharge_position

    def execute(self, object):
        try:
            result = self.proxy(self.req_discharge_position)
            HumanCollaborationTool.loginfo("Detection Result")
            HumanCollaborationTool.loginfo(result.task_command_id)
            self.task_id = result.task_command_id
            return True
        except HumanCollaborationTool.ServiceException as e:
            HumanCollaborationTool.loginfo("ServiceException : %s" % e)
            return False

#サーバーに周辺環境セット指令を出すクラス.
class PeripheralEnvironmentAreaSetClient(HumanCollaborationClient):
    def __init__(self):
        super().__init__('per_env_det_service', MonitoringAreaSetup)
        self.req_list = AreaSetupList()
        self.area_setup_result = SetupResult()
    
    def set_peripheral_area(self, sx, sy, sz, ex, ey, ez, crange, ncrange):
        set_data = AreaSetup()
        set_data.target_area.start_point.x = sx
        set_data.target_area.start_point.y = sy
        set_data.target_area.start_point.z = sz
        set_data.target_area.end_point.x = ex
        set_data.target_area.end_point.y = ey
        set_data.target_area.end_point.z = ez
        set_data.area_division.cooperative_work_range = crange
        set_data.area_division.non_cooperative_work_range = ncrange
        self.req_list.area_setup_info.append(set_data)

    def execute(self, object):
        try:
            result = self.proxy(self.req_list)
            HumanCollaborationTool.loginfo("Area Setup Result")
            HumanCollaborationTool.loginfo(result.setup_result)
            self.area_setup_result = result.setup_result
            return True
        except HumanCollaborationTool.ServiceException as e:
            HumanCollaborationTool.loginfo("ServiceException : %s" % e)
            return False
