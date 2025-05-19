# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from enum import Enum
from forklift_msg.msg import meteorcar
import statistics
import time
from custom_msgs.msg import CmdCutPliers  # 引入訊息格式

class Action():
    def __init__(self, Subscriber):
        # cmd_vel
        self.cmd_vel = cmd_vel(Subscriber)
        self.Subscriber = Subscriber
        self.NearbySequence = Enum('NearbySequence', 'initial_turn go_straight turn_right parking ')
        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        # fork_cmd
        self.pub_fork = self.Subscriber.pub_fork
        self.fork_msg = meteorcar()
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.initial_robot_pose_x = 0.0
        self.initial_robot_pose_y = 0.0
        # AprilTag_param
        self.is_marker_pose_received = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_pose_z = 0.0
        self.marker_2d_theta = 0.0
        self.initial_marker_pose_x = 0.0
        self.initial_marker_pose_y = 0.0
        self.initial_marker_pose_theta = 0.0
        # Fork_param
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0
        self.fork_threshold = 0.005
        # other
        self.check_wait_time = 0
        self.is_triggered = False

        # 初始化 y_pose_history 和窗口大小
        self.y_pose_history = []
        self.moving_average_window = 5
        self.arm_control_pub = rospy.Publisher("/cmd_cut_pliers", CmdCutPliers, queue_size=10)
        # 用於儲存最新的手臂狀態
        self.current_arm_status = None
        # 訂閱 /arm_current_status 話題
        self.arm_status_sub = rospy.Subscriber("/arm_current_status", CmdCutPliers, self.arm_status_callback, queue_size=1)


    def SpinOnce(self):
        (self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
         self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_pose_z, self.marker_2d_theta)=self.Subscriber.SpinOnce()
    
    
    def update_fork(self):
        self.updownposition = self.Subscriber.SpinOnce_fork()
    
    def fork_updown_finetune(self, desired_updownposition, fork_threshold):
        self.update_fork()
        
        while(abs(self.updownposition - desired_updownposition) > fork_threshold ):
            if self.updownposition < desired_updownposition - fork_threshold:
                
                self.pub_fork.publish(self.forkmotion.up.value)
                rospy.sleep(0.2)
                self.pub_fork.publish(self.forkmotion.stop.value)
            elif self.updownposition > desired_updownposition + fork_threshold:
                
                self.pub_fork.publish(self.forkmotion.down.value)
                rospy.sleep(0.05)
                self.pub_fork.publish(self.forkmotion.stop.value)
            rospy.sleep(0.7)
            self.update_fork()
        print(self.updownposition, desired_updownposition)


    def fnForkUpdown(self, desired_updownposition):#0~2.7
        # rospy.loginfo('updownposition: {0}'.format(self.updownposition))
        # rospy.loginfo('desired_updownposition: {0}'.format(desired_updownposition))
        # rospy.loginfo('fork_threshold: {0}'.format(self.fork_threshold))
        if(desired_updownposition < 0):
            return True
        
        self.update_fork()
        if self.updownposition < desired_updownposition - self.fork_threshold:
            self.fork_msg.fork_velocity = 2000.0
            self.pub_fork.publish(self.fork_msg)
            return False
        elif self.updownposition > desired_updownposition + self.fork_threshold:
            self.fork_msg.fork_velocity = -2000.0
            self.pub_fork.publish(self.fork_msg)
            return False
        else:
            self.fork_msg.fork_velocity = 0.0
            self.pub_fork.publish(self.fork_msg)
            return True
    

    def fork_forwardback(self, desired_forwardbackpostion):# 0~0.7
        self.update_fork()
        if self.forwardbackpostion < desired_forwardbackpostion - self.fork_threshold*1.5:
            self.pub_fork.publish(self.forkmotion.forward.value)
            return False
        elif self.forwardbackpostion > desired_forwardbackpostion + self.fork_threshold*1.5:
            self.pub_fork.publish(self.forkmotion.backword.value)
            return False
        else:
            self.pub_fork.publish(self.forkmotion.stop.value)
            return True
        
    def fnRotateToRelativeLine(self, distance, Kp, v):
        time_needed = abs(distance / (Kp * v))   # 計算所需的行駛時間
        start_time = rospy.Time.now().secs  # 獲取當前時間（秒）
        rospy.loginfo(f'time_needed:{time_needed}')
        # 開始移動
        while (rospy.Time.now().secs) < (start_time + time_needed):
            self.cmd_vel.fnGoStraight(Kp, v)
            rospy.sleep(0.1)  # 每 0.1 秒發送一次指令
        self.cmd_vel.fnStop()   # 停止機器人
        return True

    def fnseqDeadReckoningAngle_Time(self, target_angle, Kp, theta):
        target_angle_rad = math.radians(target_angle)   # 計算目標角度（弧度）
        time_needed = target_angle_rad / (Kp * theta)    # 計算所需的行駛時間
        start_time = rospy.Time.now().secs  # 獲取當前時間（秒）
        rospy.loginfo(f'time_needed:{time_needed}')
        while (rospy.Time.now().secs) < (start_time + time_needed):
            self.cmd_vel.fnTurn(Kp, theta)
            rospy.sleep(0.1)  # 每 0.1 秒發送一次指令
        self.cmd_vel.fnStop()   # 停止機器人
        return True
    
    def fnseqDeadReckoningAngle(self, target_angle):
        self.SpinOnce()  # 確保獲取到最新位置
        Kp = 0.2
        threshold = 0.015  # 停止的閾值（弧度）
        target_angle_rad = math.radians(target_angle)   # 將目標角度轉換為弧度
        if not self.is_triggered:   # 初始化：如果是第一次調用，記錄初始累積角度
            self.is_triggered = True
            self.initial_total_theta = self.robot_2d_theta  # 使用累積的總角度作為初始角度
        
        current_angle = self.robot_2d_theta - self.initial_total_theta  # 計算當前已旋轉的角度
        remaining_angle = target_angle_rad - current_angle  # 計算剩餘的旋轉角度
        if abs(remaining_angle) < threshold:   # 判斷是否達到目標角度
            self.cmd_vel.fnStop()  # 停止機器人
            self.is_triggered = False  # 重置觸發狀態
            return True
        else:
            self.cmd_vel.fnTurn(Kp, remaining_angle)    # 執行旋轉，正負值決定方向
            return False

    def fnseqDeadReckoning(self, dead_reckoning_dist):  # 使用里程計算移動到指定距離
        self.SpinOnce()  # 確保獲取到最新位置
        Kp = 0.2
        threshold = 0.015  # 停止的閾值
        if self.is_triggered == False:  # 如果還沒啟動，記錄初始位置
            self.is_triggered = True
            self.initial_robot_pose_x = self.robot_2d_pose_x
            self.initial_robot_pose_y = self.robot_2d_pose_y
        # 計算當前移動距離
        current_dist = self.fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)
        # 計算剩餘距離
        remaining_dist = dead_reckoning_dist - math.copysign(1, dead_reckoning_dist) * current_dist
        # 判斷是否達到目標距離
        if abs(remaining_dist) < threshold:  # 進入停止條件
            self.cmd_vel.fnStop()
            self.is_triggered = False
            return True
        else:
            # 計算速度並保持方向
            self.cmd_vel.fnGoStraight(Kp, remaining_dist)
            return False
            
    def fnSeqChangingDirection(self, desired_angle, object_name):
        self.SpinOnce()
        Kp = 0.02
        desired_angle_turn = math.atan2(self.marker_2d_pose_y, self.marker_2d_pose_x)
        if self.TFConfidence(object_name):
            if desired_angle_turn <0:
                desired_angle_turn = desired_angle_turn + math.pi
            else:
                desired_angle_turn = desired_angle_turn - math.pi

            self.cmd_vel.fnTurn(Kp, desired_angle_turn)
            
            if abs(desired_angle_turn) < desired_angle  :
                self.cmd_vel.fnStop()
                if self.check_wait_time > 20 :
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  +1
                    return False
            else:
                self.check_wait_time =0
                return False
        else:
            # rospy.logwarn("Confidence Low")
            return False

    def fnSeqChangingtheta(self, threshod, object_name): #旋轉到marker的theta值為0, threshod為角度誤差值
        self.SpinOnce()
        Kp = 0.02
        if self.TFConfidence(object_name):
            # self.marker_2d_theta= self.TrustworthyMarker2DTheta(3)
            # print("desired_angle_turn", self.marker_2d_theta)
            # print("threshod", threshod)
            if abs(self.marker_2d_theta) < threshod  :
                self.cmd_vel.fnStop()
                if self.check_wait_time > 20 :
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  + 1
                    return False
            else:
                self.cmd_vel.fnTurn(Kp, -self.marker_2d_theta)
                self.check_wait_time = 0
                return False
        else:
            return False
        
        
    def fnSeqMovingNearbyParkingLot(self,desired_dist_threshold):
        self.SpinOnce()
        Kp = 0.2
        if self.current_nearby_sequence == self.NearbySequence.initial_turn.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

                self.initial_marker_pose_theta = self.TrustworthyMarker2DTheta(3)
                self.initial_marker_pose_x = self.marker_2d_pose_x
                # print("initial_marker_pose_theta ", self.initial_marker_pose_theta)
                # decide doing fnSeqMovingNearbyParkingLot or not
                desired_dist = -1* self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))
                if abs(desired_dist) < desired_dist_threshold:
                    self.is_triggered = False
                    return True
            
            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)
            

            
            desired_angle_turn = -1. * desired_angle_turn
            self.cmd_vel.fnTurn(Kp, desired_angle_turn)

            if abs(desired_angle_turn) < 0.03:
                self.cmd_vel.fnStop()
                if self.check_wait_time >15:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.check_wait_time =self.check_wait_time +1
            elif abs(desired_angle_turn) < 0.045 and self.check_wait_time :
                self.cmd_vel.fnStop()
                if self.check_wait_time > 15:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.check_wait_time =self.check_wait_time +1
            else:
                self.check_wait_time =0    

        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

            dist_from_start = self.fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)
            desired_dist = self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))

            remained_dist = desired_dist + dist_from_start
            self.cmd_vel.fnGoStraight(Kp, -desired_dist)

            if abs(remained_dist) < 0.07:
                self.cmd_vel.fnStop()
                self.current_nearby_sequence = self.NearbySequence.turn_right.value
                self.is_triggered = False


        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            
            self.cmd_vel.fnTurn(Kp, -desired_angle_turn)

            if abs(desired_angle_turn) < 0.01:
                self.cmd_vel.fnStop()
                self.is_triggered = False
                return True
            
            # if abs(desired_angle_turn) < 0.03:
            #     self.cmd_vel.fnStop()
            #     if self.check_wait_time > 20:
            #         self.check_wait_time = 0
            #         self.current_nearby_sequence = self.NearbySequence.parking.value
            #         self.is_triggered = False
            #         return True                
            #     else:
            #         self.check_wait_time =self.check_wait_time  +1
            # elif abs(desired_angle_turn) < 0.045 and self.check_wait_time:
            #     self.cmd_vel.fnStop()
            #     if self.check_wait_time > 20:
            #         self.check_wait_time = 0
            #         self.current_nearby_sequence = self.NearbySequence.parking.value
            #         self.is_triggered = False
            #         return True                
            #     else:
            #         self.check_wait_time =self.check_wait_time  +1
            # else:
            #     self.check_wait_time =0    
        return False

    def fnSeqParking(self, parking_dist, kp, object_name):
        self.SpinOnce()
        desired_angle_turn = math.atan2(self.marker_2d_pose_y - 0, self.marker_2d_pose_x - 0)
        if self.TFConfidence(object_name):

            if desired_angle_turn <0:
                desired_angle_turn = desired_angle_turn + math.pi
            else:
                desired_angle_turn = desired_angle_turn - math.pi


            self.cmd_vel.fnTrackMarker(desired_angle_turn, kp)

            if (abs(self.marker_2d_pose_x) < parking_dist)  :
                self.cmd_vel.fnStop()
                if self.check_wait_time > 15:
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  +1
            elif (abs(self.marker_2d_pose_x) < parking_dist) and self.check_wait_time:
                self.cmd_vel.fnStop()
                if self.check_wait_time > 15:
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  +1
            else:
                self.check_wait_time =0
                return False
        else:
            return False
        
    def fnSeqdecide(self, decide_dist):#decide_dist偏離多少公分要後退
        self.SpinOnce()
        dist = self.marker_2d_pose_y
        if  abs(dist) < abs(decide_dist):
            return True
        else:
            return False

    def fnseqMoveToMarkerDist(self, marker_dist): #(使用marker)前後移動到距離marker_dist公尺的位置
        self.SpinOnce()
        Kp = 0.2
        if(abs(marker_dist) < 2.0):
            threshold = 0.015
        else:
            threshold = 0.03

        dist = math.sqrt(self.marker_2d_pose_x**2 + self.marker_2d_pose_y**2)
        
        if dist < (abs(marker_dist)-threshold):
            self.cmd_vel.fnGoStraight(Kp, marker_dist - dist)
            return False
        elif dist > (abs(marker_dist)+threshold):
            self.cmd_vel.fnGoStraight(Kp, marker_dist - dist)
            return False
        else:
            self.cmd_vel.fnStop()
            return True
            
    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def TrustworthyMarker2DTheta(self, time):
        marker_2d_theta_list = [0.0]
        initial_time = rospy.Time.now().secs
        
        while(abs(initial_time - rospy.Time.now().secs) < time):
            self.SpinOnce()
            marker_2d_theta_list.append(self.marker_2d_theta)
            # print("self.marker_2d_theta", self.marker_2d_theta)
            rospy.sleep(0.05)
        # print("marker_2d_theta_list", marker_2d_theta_list)
        threshold = 0.5
        mean = statistics.mean(marker_2d_theta_list)
        stdev = statistics.stdev(marker_2d_theta_list)
        upcutoff = mean + threshold * stdev
        downcutoff = mean - threshold * stdev
        clean_list = []
        for i in marker_2d_theta_list:
            if(i > downcutoff and i < upcutoff):
               clean_list.append(i)
               
        return statistics.median(clean_list) 

    def TFConfidence(self, object_name):#判斷TF是否可信
        # rospy.loginfo('shelf_detection: {0}'.format(self.Subscriber.sub_detectionConfidence.shelf_detection))
        # rospy.loginfo('shelf_confidence: {0}'.format(self.Subscriber.sub_detectionConfidence.shelf_confidence))
        # rospy.loginfo('confidence_minimum: {0}'.format(self.Subscriber.confidence_minimum))
        if object_name == "bodycamera":
            if (not self.Subscriber.sub_detectionConfidence.pallet_detection) or self.Subscriber.sub_detectionConfidence.pallet_confidence < self.Subscriber.confidence_minimum:
                self.cmd_vel.fnStop()
                return False
        elif object_name == "bodycamera":
            if (not self.Subscriber.sub_detectionConfidence.shelf_detection) or self.Subscriber.sub_detectionConfidence.shelf_confidence < self.Subscriber.confidence_minimum:
                self.cmd_vel.fnStop()
                return False
        return True

#-------------------------------------------水果採摘控制----------------------------------------------------

    def refine_alignment(self, object_name, target_y=0.007, max_iterations=10, threshold=0.006):
        """
        當水果位於相機的左/右（以 y 軸衡量）時，對底盤做小幅微調，每次移動後檢查是否進入範圍。
        """
        rospy.sleep(5)         # 避免發送頻率過高

        Y_MIN = -0.002  # 允許的最小值
        Y_MAX = target_y  # 允許的最大值

        prev_y = None  # 用來追蹤上一個 y 值，確保有更新

        for i in range(max_iterations):
            self.SpinOnce()

            if not self.TFConfidence(object_name):
                self.cmd_vel.fnStop()
                rospy.logwarn(f"TF Data Not Confident for object '{object_name}' - Stopping")
                return False

            smoothed_y = self.compute_moving_average(self.marker_2d_pose_y)
            error = smoothed_y - target_y
            rospy.loginfo(f"[refine_alignment] Iter {i+1}, Camera Y = {smoothed_y:.6f}, Error = {error:.6f}")

            # 防止數據未更新，等姿態更新
            if prev_y is not None and abs(smoothed_y - prev_y) < 0.00001:
                rospy.logwarn("Pose not updated, waiting for new data...")
                rospy.sleep(4)
                continue

            prev_y = smoothed_y

            # 如果已經在範圍內，立即停止並返回
            if Y_MIN <= smoothed_y <= Y_MAX:
                self.cmd_vel.fnStop()
                rospy.loginfo(f"Camera Y = {smoothed_y:.6f} is within range [{Y_MIN}, {Y_MAX}], alignment complete!")
                return True

            # 不在範圍內，進行修正
            if smoothed_y > Y_MAX:
                self.cmd_vel.fnGoStraight_fruit()
                rospy.loginfo("Over threshold, moving backward to correct.")
            elif smoothed_y < Y_MIN:
                self.cmd_vel.fnGoBack()
                rospy.loginfo("Under threshold, moving forward to correct.")

            # 每次移動後立即停止，等數據更新並檢查
            move_duration = 0.05  # 進一步縮短移動時間，每次移動更小距離
            rospy.sleep(move_duration)
            self.cmd_vel.fnStop()
            rospy.loginfo("Stop, waiting for pose update...")

            # 動態等待數據更新
            prev_y_temp = smoothed_y
            timeout = 10.0  # 最多等待 10 秒
            start_time = time.time()
            while time.time() - start_time < timeout:
                self.SpinOnce()
                smoothed_y = self.compute_moving_average(self.marker_2d_pose_y)
                if abs(smoothed_y - prev_y_temp) > 0.00001:  # 數據已更新
                    break
                rospy.sleep(0.1)
            rospy.loginfo(f"Pose updated, new Y = {smoothed_y:.6f}")

            # 移動後立即檢查是否進入範圍
            if Y_MIN <= smoothed_y <= Y_MAX:
                self.cmd_vel.fnStop()
                rospy.loginfo(f"Camera Y = {smoothed_y:.6f} is within range [{Y_MIN}, {Y_MAX}] after move, alignment complete!")
                return True

        self.cmd_vel.fnStop()
        rospy.logwarn("Failed to Align Within Max Iterations")
        return False

    def blind_walk_backward(self, duration, speed=-0.2):
        """
        讓機器人以固定速度向後盲走 `duration` 秒，
        並在結束後停止，然後返回 True。
        """
        rospy.loginfo(f"🚀 開始盲走往後 {duration} 秒，速度 {speed} m/s")

        # 先停一下，避免累積舊指令
        self.cmd_vel.fnStop()
        rospy.sleep(0.1)  # 確保停止指令生效

        start_time = time.time()

        # 在 `duration` 秒內持續發送後退指令
        # 並在結束時停止機器人
        while (time.time() - start_time) < duration and not rospy.is_shutdown():
            self.cmd_vel.fnGoBack2()  # 持續發送後退指令
            rospy.sleep(0.05)         # 避免發送頻率過高

        self.cmd_vel.fnStop()
        return True


    def fnControlArm(self, height, timeout=8.0):
        """
        控制機械手臂的高度、長度和爪子開合狀態，
        並持續檢查手臂當前狀態是否已達到指定目標，
        如果手臂狀態與目標在允許誤差範圍內則返回 True，
        否則在 timeout 時間內仍未達標則返回 False。

        :param height: 目標高度 (毫米)
        :param length: 目標伸長長度 (毫米)
        :param claw_state: 目標爪子狀態 (True 表示閉合, False 表示張開)
        :param timeout: 等待超時秒數 (預設 5 秒)
        :return: 如果在 timeout 內手臂狀態與目標在允許誤差內則返回 True，否則返回 False
        """
        arm_cmd = CmdCutPliers()
        arm_cmd.height1 = height
        # arm_cmd.length1 = length
        arm_cmd.claw1 = True
        arm_cmd.enable_motor1 = True  # 啟動手臂馬達
        self.arm_control_pub.publish(arm_cmd)


        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.current_arm_status is not None:
                current_height = self.current_arm_status.height1
                current_length = self.current_arm_status.length1
                # 假設手臂回傳的 claw 狀態為數值，非 0 表示 True
                current_claw = bool(self.current_arm_status.claw1)

                if (abs(abs(current_height) - height) <= 10) :
                    rospy.loginfo("Arm reached target state.")
                    return True
            else:
                rospy.logwarn("尚未接收到手臂狀態訊息。")
            # rospy.sleep(1)  # 每 100ms 檢查一次
        rospy.logwarn("Timeout waiting for arm to reach target state.")
        return False



    def arm_status_callback(self, msg):
        """
        當收到 /arm_current_status 的消息時更新內部變數
        """
        self.current_arm_status = msg
        # rospy.loginfo("Received arm status: height1=%d, length1=%d, claw1=%s" %(msg.height1, msg.length1, str(msg.claw1)))
 


    def fnControlArmBasedOnFruitZ(self, object_name, timeout=10.0, tolerance=4):
            """
            根據水果的 Z 軸數值持續調整手臂高度，
            當水果的 Z 值進入允許範圍 (lower_z ~ upper_z) 時，認為已達標停止調整，
            並保持手臂長度不變。
            參數從 self.Subscriber 讀取。
            """
            start_time = time.time()
            while time.time() - start_time < timeout:
                # 更新水果 Z 與手臂狀態
                self.SpinOnce()  # 更新 self.marker_2d_pose_z 與 self.current_arm_status
                fruit_z = self.marker_2d_pose_z

                if self.current_arm_status is None:
                    rospy.logwarn("尚未接收到手臂狀態訊息。")
                    rospy.sleep(0.5)
                    continue

                # 將下位機回傳的手臂高度（可能為負）轉換為正數
                current_height = abs(self.current_arm_status.height1)
                current_length = self.current_arm_status.length1  # 前伸長度保持不變

                # 取得信心指數
                confidence = self.TFConfidence(object_name)
                if confidence is None or confidence < 0.5:
                    rospy.logwarn(f"信心指數不足 ({confidence}), 暫停調整，等待重新估測。")
                    rospy.sleep(1.0)
                    continue

                rospy.loginfo(
                    f"當前水果 Z 值: {fruit_z:.4f}, 允許範圍: ({self.Subscriber.cut_pliers_lower_z} ~ {self.Subscriber.cut_pliers_upper_z}), 當前手臂高度: {current_height}"
                )

                # 若水果 Z 值在目標範圍內，則認為達標，停止調整
                if self.Subscriber.cut_pliers_lower_z <= fruit_z <= self.Subscriber.cut_pliers_upper_z:
                    rospy.loginfo("✅ 水果 Z 值已達標，停止調整高度。")
                    return True

                # 根據水果 Z 值決定調整方向：
                # 若水果 Z 值低於 lower_z，水果太低，需上升 => 手臂高度增加
                if fruit_z < self.Subscriber.cut_pliers_lower_z:
                    new_height = current_height + abs(self.Subscriber.cut_pliers_height_increment)
                    rospy.loginfo(f"📈 水果過低，預計上升：{current_height} -> {new_height} mm")
                # 若水果 Z 值高於 upper_z，水果太高，需下降 => 手臂高度減少
                else:
                    new_height = current_height - abs(self.Subscriber.cut_pliers_height_increment)
                    rospy.loginfo(f"📉 水果過高，預計下降：{current_height} -> {new_height} mm")

                # 限制新高度在 [min_height, max_height] 範圍內
                new_height = max(min(new_height, self.Subscriber.cut_pliers_max_height), self.Subscriber.cut_pliers_min_height)
                rospy.loginfo(f"最終設定高度: {new_height} mm (範圍 [{self.Subscriber.cut_pliers_min_height}, {self.Subscriber.cut_pliers_max_height}])")

                # 若高度變化超過容許誤差則發布控制命令
                if abs(new_height - current_height) > tolerance:
                    msg = CmdCutPliers()
                    msg.height1 = int(new_height)    # 發布正值高度
                    msg.length1 = int(current_length)  # 保持前伸長度不變
                    msg.enable_motor1 = True
                    msg.enable_motor2 = True
                    msg.target_motor = 0  # target_motor = 0 表示控制高度
                    msg.motor_value = int(new_height)  # 馬達值設定為目標高度
                    self.arm_control_pub.publish(msg)
                    rospy.loginfo(f"✅ 發送手臂控制指令: 高度={new_height}, 長度={current_length}")
                else:
                    rospy.loginfo("高度變化小於容許誤差，避免重複發布指令。")

                # 等待手臂達到新高度
                reach_start = time.time()
                while time.time() - reach_start < 5:
                    self.SpinOnce()
                    # 讀取時也將高度轉換為正數進行比較
                    current_height = abs(self.current_arm_status.height1)
                    error = abs(current_height - new_height) + 1
                    if error <= tolerance:
                        rospy.loginfo(
                            f"✅ 手臂調整成功：當前高度 {current_height} mm (目標 {new_height} mm，誤差 {error} mm)"
                        )
                        break
                    else:
                        rospy.logwarn(
                            f"⏳ 當前高度 {current_height} mm，目標 {new_height} mm，誤差 {error} mm，等待中..."
                        )
                        rospy.sleep(1)
                
                rospy.sleep(1)  # 給予一些時間讓數據穩定後再重新評估水果 Z 值

            rospy.logwarn("❌ 超時：手臂未能調整至符合目標水果 Z 範圍。")
            return False

    def fnControlArmBasedOnFruitX(self, object_name, timeout=10.0):
            """
            根據水果的 x 軸數值持續調整手臂前伸長度，
            當水果的 x 值大於 target_x 時，認為已達標停止調整，
            並保持手臂高度不變。
            參數從 self.Subscriber 讀取。
            """
            start_time = time.time()
            
            if not hasattr(self, "last_valid_length"):
                self.last_valid_length = 0  # 確保變數初始化

            while time.time() - start_time < timeout:
                # 更新水果 x 軸資訊
                self.SpinOnce()
                fruit_x = self.marker_2d_pose_x
                rospy.loginfo(
                    f"當前水果 X 值: {fruit_x:.4f}, 目標: {self.Subscriber.cut_pliers_target_x:.4f}, 前伸長度: {self.current_arm_status.length1}"
                )

                # 信心指數檢查
                confidence = self.TFConfidence(object_name)
                if confidence is None or confidence < 0.5:
                    rospy.logwarn(f"信心指數不足 ({confidence}), 停止手臂前伸。")
                    return False

                # 若水果 x 值已達目標，則返回 True
                if fruit_x >= self.Subscriber.cut_pliers_target_x:
                    rospy.loginfo("水果 X 值已達標，停止前伸。")
                    return True

                # 確保 `length1` 只增不減
                current_length = self.current_arm_status.length1

                # 如果 `length1=0`，使用上次有效值
                if current_length == 0:
                    rospy.logwarn(f"⚠ `length1=0`，忽略此數值，保持 {self.last_valid_length} mm")
                    current_length = self.last_valid_length
                elif current_length < self.last_valid_length:
                    rospy.logwarn(f"⚠ `length1` 變小 ({current_length} mm)，恢復到 {self.last_valid_length} mm")
                    current_length = self.last_valid_length
                else:
                    self.last_valid_length = current_length  # 記錄最後一次的有效長度

                # 設定新的目標長度
                target_length = min(current_length + self.Subscriber.cut_pliers_length_increment, self.Subscriber.cut_pliers_max_length)
                rospy.loginfo(f"嘗試前伸: {target_length} mm")

                # 發送控制命令
                msg = CmdCutPliers()
                msg.height1 = int(self.current_arm_status.height1)  # 保持當前高度
                msg.length1 = int(target_length)  # 設定前伸長度
                msg.enable_motor1 = True
                msg.enable_motor2 = True
                msg.target_motor = 1
                msg.motor_value = int(target_length)

                self.arm_control_pub.publish(msg)

                # 等待手臂到達目標長度
                rospy.loginfo(f"等待手臂到達長度: {target_length} mm")
                reach_start_time = time.time()
                while time.time() - reach_start_time < 5:  # 最多等待 5 秒
                    self.SpinOnce()
                    current_length = self.current_arm_status.length1

                    # 確保 `length1` 只增加
                    if current_length == 0:
                        rospy.logwarn(f"⚠ `length1=0`，忽略此數值，保持 {self.last_valid_length} mm")
                        current_length = self.last_valid_length
                    elif current_length < self.last_valid_length:
                        rospy.logwarn(f"⚠ `length1` 變小 ({current_length} mm)，恢復到 {self.last_valid_length} mm")
                        current_length = self.last_valid_length
                    else:
                        self.last_valid_length = current_length  # 更新最後一次的有效長度

                    if abs(current_length - target_length) <= 10:  # 允許 10mm 誤差
                        rospy.loginfo(f"✅ 手臂已到達目標長度 {current_length} mm")
                        break
                    else:
                        rospy.logwarn(f"⏳ 目前長度 {current_length} mm，目標 {target_length} mm，等待中...")
                        rospy.sleep(0.5)  # 每 500ms 檢查一次

                rospy.sleep(1)  # 延長等待時間，確保 `length1` 更新穩定

            rospy.logwarn("Timeout: 手臂未能達到目標 X 值。")
            return False


    def fnBlindExtendArm(self, timeout=7.0):
            """
            盲伸手臂：在當前長度的基礎上，額外前伸 extra_length，並在到達後閉合剪鉗
            參數從 self.Subscriber 讀取。
            """
            # 新增變數，確保這個函數不會被重複執行
            if hasattr(self, "blind_extend_completed") and self.blind_extend_completed:
                rospy.logwarn("⚠ `fnBlindExtendArm()` 已執行過，跳過此次呼叫")
                return False  # 防止再次執行

            start_time = time.time()

            # 取得當前長度
            current_length = self.current_arm_status.length1
            if current_length is None:
                rospy.logerr("❌ 無法獲取當前手臂長度，盲伸失敗")
                return False

            self.last_valid_length = current_length  # 更新最後一次的有效長度

            # 設定目標長度
            target_length = min(current_length + self.Subscriber.cut_pliers_blind_extend_length, self.Subscriber.cut_pliers_max_length)
            rospy.loginfo(f"🔵 盲伸: 當前長度={current_length} mm, 目標長度={target_length} mm")

            # 發送控制指令
            msg = CmdCutPliers()
            msg.height1 = int(self.current_arm_status.height1)  # 保持當前高度
            msg.length1 = int(target_length)  # 設定新的前伸長度
            msg.enable_motor1 = True
            msg.enable_motor2 = True
            msg.target_motor = 1
            msg.motor_value = int(target_length)

            self.arm_control_pub.publish(msg)

            # 等待手臂到達目標長度
            rospy.loginfo(f"⏳ 等待手臂到達長度 {target_length} mm")
            while time.time() - start_time < timeout:
                self.SpinOnce()
                current_length = self.current_arm_status.length1

                self.last_valid_length = current_length  # 更新最後一次的有效長度

                if abs(current_length - target_length) <= 10:  # 允許 10 mm 誤差
                    rospy.loginfo(f"✅ 手臂已成功盲伸至 {current_length} mm")
                    # 設置變數，防止重複執行
                    self.blind_extend_completed = True
                    return True

                rospy.sleep(0.5)

            rospy.logerr(f"⏰ 盲伸超時: 目標 {target_length} mm 未達成，當前 {current_length} mm")
            return False

    # def fnControlClaw(self, claw_state, timeout=3):
    #     """
    #     控制剪鉗的開合 (claw1)，並等待其完成

    #     :param claw_state: True = 閉合剪鉗, False = 打開剪鉗
    #     :param timeout: 等待剪鉗動作完成的最大時間 (秒)
    #     :return: True 若剪鉗成功執行, False 若超時或發生錯誤
    #     """
    #     start_time = time.time()

    #     # 確保 claw_state 為 bool
    #     claw_state = bool(claw_state)

    #     # 發送剪鉗控制指令
    #     msg = CmdCutPliers()
    #     msg.height1 = self.current_arm_status.height1  # 保持當前高度
    #     msg.length1 = self.current_arm_status.length1  # 保持當前長度
    #     msg.claw1 = claw_state  # 確保為 bool
    #     msg.enable_motor1 = True
    #     msg.enable_motor2 = True

    #     self.arm_control_pub.publish(msg)
    #     rospy.loginfo(f"✂ 剪鉗指令發送: {'閉合' if claw_state else '打開'}")

    #     # 等待剪鉗狀態變更，達到目標狀態後等待2秒再返回True
    #     while time.time() - start_time < timeout:
    #         self.SpinOnce()  # 處理 ROS 回傳的狀態
    #         if self.current_arm_status.claw1 == claw_state:
    #             rospy.loginfo(f"✅ 剪鉗 {'閉合' if claw_state else '打開'} 成功，等待2秒以穩定狀態...")
    #             rospy.sleep(10)  # 等待2秒
    #             return True
    #         rospy.logwarn(f"⏳ 剪鉗動作中... 目標: {claw_state}, 當前: {self.current_arm_status.claw1}")
    #         rospy.sleep(0.1)
        
    #     rospy.logerr(f"⏰ 剪鉗動作超時: 目標 {claw_state}, 當前 {self.current_arm_status.claw1}")
    #     return False

    def fnControlClaw(self, claw_state, timeout=3):
        """
        控制剪鉗的開合 (claw1)，並等待其完成

        :param claw_state: True = 閉合剪鉗, False = 打開剪鉗
        :param timeout: 等待剪鉗動作完成的最大時間 (秒)
        :return: True 若剪鉗成功執行, False 若超時或發生錯誤
        """
        start_time = time.time()

        # 確保 claw_state 為 bool
        claw_state = bool(claw_state)

        # 等待初始手臂狀態
        while self.current_arm_status is None and time.time() - start_time < 1.0:
            rospy.logwarn("等待手臂狀態初始化...")
            rospy.sleep(0.1)
        if self.current_arm_status is None:
            rospy.logerr("❌ 未接收到手臂狀態，無法控制剪鉗")
            return False

        # 發送剪鉗控制指令
        msg = CmdCutPliers()
        msg.height1 = self.current_arm_status.height1  # 保持當前高度
        msg.length1 = self.current_arm_status.length1  # 保持當前長度
        msg.claw1 = claw_state  # 確保為 bool
        msg.enable_motor1 = True
        msg.enable_motor2 = True
        self.arm_control_pub.publish(msg)
        rospy.loginfo(f"✂ 剪鉗指令發送: {'閉合' if claw_state else '打開'}")

        # 等待剪鉗狀態變更
        while time.time() - start_time < timeout:
            self.SpinOnce()  # 處理 ROS 回傳的狀態
            if self.current_arm_status.claw1 == claw_state:
                if claw_state:  # 閉合
                    rospy.loginfo(f"✅ 剪鉗閉合成功，等待2秒以穩定狀態...")
                    rospy.sleep(5)  # 閉合後等待2秒
                else:  # 打開
                    rospy.loginfo(f"✅ 剪鉗打開成功，等待10秒以穩定狀態...")
                    rospy.sleep(25)  # 打開後等待10秒
                return True
            rospy.logwarn(f"⏳ 剪鉗動作中... 目標: {claw_state}, 當前: {self.current_arm_status.claw1}")
            rospy.sleep(0.1)
        
        rospy.logerr(f"⏰ 剪鉗動作超時: 目標 {claw_state}, 當前: {self.current_arm_status.claw1}")
        return False


    def fnRetractArm(self, timeout=12.0):
            """
            後退手臂到指定的目標長度。
            參數從 self.Subscriber 讀取。
            """
            if hasattr(self, "retract_executed") and self.retract_executed:
                rospy.logwarn("⚠ 已執行過後退，忽略此次請求")
                return False

            target_length_1 = self.Subscriber.cut_pliers_retract_length
            rospy.loginfo(f"📢 正在執行 fnRetractArm(), 目標長度: {target_length_1}")

            start_time = time.time()
            current_length = self.current_arm_status.length1

            if current_length is None:
                rospy.logerr("❌ 無法獲取當前手臂長度，後退失敗")
                return False

            if target_length_1 > current_length:
                rospy.logwarn(f"⚠ 目標長度 {target_length_1} mm 大於當前長度 {current_length} mm，忽略請求")
                return False

            # 設置為已執行後退
            self.retract_executed = True

            # 發送後退訊息
            msg = CmdCutPliers()
            msg.height1 = int(self.current_arm_status.height1)
            msg.length1 = int(target_length_1)
            msg.claw1 = int(self.current_arm_status.claw1)
            msg.enable_motor1 = True
            msg.enable_motor2 = True
            msg.mode = 1  # 後退模式
            
            self.arm_control_pub.publish(msg)
            rospy.loginfo(f"🔵 已發送後退指令: {msg}")

            while time.time() - start_time < timeout:
                self.SpinOnce()
                current_length = self.current_arm_status.length1

                if abs(current_length - target_length_1) <= 10:
                    rospy.loginfo(f"✅ 手臂已成功縮回至 {current_length} mm")
                    return True

                rospy.logwarn(f"⏳ 目前長度 {current_length} mm，目標 {target_length_1} mm，等待中...")
                rospy.sleep(0.5)

            rospy.logerr(f"⏰ 手臂後退超時: 目標 {target_length_1} mm 未達成，當前 {current_length} mm")
            return False


    def compute_moving_average(self, new_value):
        """
        計算滑動平均值。
        """
        # 將新數值加入歷史紀錄
        self.y_pose_history.append(new_value)

        # 若歷史數據超過窗口大小，移除最舊數據
        if len(self.y_pose_history) > self.moving_average_window:
            self.y_pose_history.pop(0)

        # 計算平均值
        return sum(self.y_pose_history) / len(self.y_pose_history)



class cmd_vel():
    def __init__(self, Subscriber):
        self.Subscriber = Subscriber
        self.pub_cmd_vel = self.Subscriber.pub_cmd_vel
        self.front = False

    def cmd_pub(self, twist):
        if not self.front:
            twist.linear.x = -twist.linear.x

        if twist.angular.z > 0.2:
            twist.angular.z =0.2
        elif twist.angular.z < -0.2:
            twist.angular.z =-0.2
        if twist.linear.x > 0 and twist.linear.x < 0.02:
            twist.linear.x =0.02
        elif twist.linear.x < 0 and twist.linear.x > -0.02:
            twist.linear.x =-0.02  

        if twist.linear.x > 0.2:
            twist.linear.x =0.2
        elif twist.linear.x < -0.2:
            twist.linear.x =-0.2                     
        if twist.angular.z > 0 and twist.angular.z < 0.05:
            twist.angular.z =0.05
        elif twist.angular.z < 0 and twist.angular.z > -0.05:
            twist.angular.z =-0.05
        self.pub_cmd_vel.publish(twist)

    def fnStop(self):
        twist = Twist()
        self.cmd_pub(twist)

    def fnTurn(self, Kp=0.2, theta=0.):
        # Kp = 0.3 #1.0
        twist = Twist()
        twist.angular.z = Kp * theta
        self.cmd_pub(twist)

    def fnGoStraight(self, Kp=0.2, v=0.):
        twist = Twist()
        twist.linear.x = Kp * v

        self.cmd_pub(twist)

    def fnGoBack(self):
        twist = Twist()
        twist.linear.x = -0.02
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.cmd_pub(twist)

    def fnfork(self,direction):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = direction
        twist.angular.z = 0

        self.cmd_pub(twist)


    def fnTrackMarker(self, theta, kp):
        # Kp = 4.0 #6.5

        twist = Twist()
        twist.linear.x = 0.05
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0

        twist.angular.z = kp * theta
        self.cmd_pub(twist)

    def fnGoStraight_fruit(self):      #控制叉車前進
        twist = Twist()
        twist.linear.x = 0.01
        self.cmd_pub(twist)

  
    def fnGoBack2(self):      #控制叉車前進
        twist = Twist()
        twist.linear.x = -0.08
        self.cmd_pub(twist)