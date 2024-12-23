#!/usr/bin/env python3
# coding: UTF-8

from EnumerateModule import EnumEvent
from HumanCollaborationCommunicationModule import *
from HumanCollaborationUserDefineModule import *
from HumanCollaborationEventModule import HumanCollaborationTaskCommandList

from smach import State
from peripheral_environment_detection.msg import *
from TransitionModule import Transition
from HumanCollaborationToolModule import HumanCollaborationTool

#状態遷移イベント抽象クラス.
class HumanCollaborationState(State):
    """
    Base class for HumanCollaboration state.

    Inherit this class and set the appropriate EnumState.

    Attributes
    ----------
    history_label : str
        state name
    """
    history_label = None

    def __init__(self, label, tran:Transition,  outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        """constructor

        Parameters
        ----------
        label : str
            state name
        tran : Trsansition object
            state transition and state value, event value
        outcomes  : array of string
            Custom outcomes for this state. 
        """
        super().__init__(outcomes, input_keys, output_keys, io_keys)
        self.label = label
        self.tran = tran

    @classmethod
    def set_history_label(cls, label):
        cls.history_label = label

    @classmethod
    def reset_history_label(cls):
        cls.set_history_label(None)

    @classmethod
    def get_history_label(cls):
        return cls.history_label

    def set_event(self, event:EnumEvent):
        return self.tran.set_event(event)

    def get_state(self):
        return self.tran.get_state()

    def preempt(self, info):
        if self.preempt_requested():
            HumanCollaborationTool.loginfo(info)
            self.service_preempt()
            return True
        else:
            return False
    
    #結果を文字列に変換.
    def return_btos(self, value : bool):
        return 'succeeded' if value else "retry"

#現在のデータを保持するクラス.
class HumanCollaborationCurrentData():
    current_state = None
    @classmethod
    def GetCurrentState(cls):
        return cls.current_state
    
    @classmethod
    def SetCurrentState(cls, state:HumanCollaborationState):
        cls.current_state = state

    current_command_list = None
    @classmethod
    def GetCurrentCommandList(cls):
        return cls.current_command_list
    
    @classmethod
    def SetCurrentCommandList(cls, command_list:HumanCollaborationTaskCommandList):
        cls.current_command_list = command_list