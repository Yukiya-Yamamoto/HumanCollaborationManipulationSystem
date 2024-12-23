#!/usr/bin/env python3
# coding: UTF-8

#####################################################################################################################
#このノードは．人協働マニピュレーションモジュールを機能単位で作成したモジュールです．　
#ハンド部分のモジュールと人検知部分の追加を行っています．　　　　　　　　　　　　　　 #
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/11/07
#08/30 排出位置検出システム編集
#09/21 排出位置検出システム　統合
#10/05 排出位置検出システム　統合確認　
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

from HumanCollaborationStateModule import HumanCollaborationStateMachine
from HumanCollaborationCommunicationModule import TaskCommandServer, TaskSuspendServer, TaskFinalServer, TaskGetStateServer, HumanCollaborationSubscriverThread, PeripheralEnvironmentAreaSetClient, DischargePositionClient, WorkDetectionClient
from HumanCollaborationEventModule import HumanCollaborationEventPublisher, HumanCollaborationEventSubscriverWorkStart
from HumanCollaborationToolModule import HumanCollaborationTool

##################################################################
#　　　　　　　　　　　 クラス定義　                            #
##################################################################
class HumanCollaboration:
    def __init__(self):
        HumanCollaborationTool.init_node('human_collaboration_node')
        self.subscthread = HumanCollaborationSubscriverThread()
        area = PeripheralEnvironmentAreaSetClient()
        disc = DischargePositionClient()
        workd = WorkDetectionClient()
        workstart_event = HumanCollaborationEventSubscriverWorkStart()
        self.statemachine = HumanCollaborationStateMachine(
              TaskCommandServer(),
              TaskSuspendServer(),
              TaskFinalServer(),
              TaskGetStateServer(),
              area,
              disc,
              workd,
              workstart_event
              )
        HumanCollaborationEventPublisher.register(self.statemachine)
        HumanCollaborationEventPublisher.register(workstart_event)

    def execute(self):
        self.subscthread.start()
        HumanCollaborationTool.loginfo('State machine Start')
        self.statemachine.execute()
        HumanCollaborationTool.loginfo(' State machine End')
        self.subscthread.join()

##################################################################
#　　　　　　　　　　　 以下実行処理内容                         #
##################################################################   
if __name__ == '__main__':
 hc = HumanCollaboration()
 HumanCollaborationTool.init_node('human_collaboration_node')
 
#実行
 hc.execute()
 
