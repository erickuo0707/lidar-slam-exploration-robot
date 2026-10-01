#!/usr/bin/env python3
import rospy, numpy as np, random
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped

class SmartExplorer:
    def __init__(self):
        rospy.init_node('smart_explorer')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.DIST_WARN  = 0.25
        self.DIST_SIDE  = 0.06
        self.FWD_SPEED  = 0.08
        self.TURN_SPEED = 0.08

        self.robot_pos = (0.0, 0.0)
        self.robot_yaw = 0.0
        self.map = None
        self.visited = []
        self.target = None
        self.target_fail_count = 0
        self.turn_start = None
        self.turn_dir = 1.0
        self.TURN_DUR = 2.0
        self.escape_start = None
        self.escape_phase = 0
        self.last_pos = (0.0, 0.0)
        self.stuck_timer = rospy.Time.now()

        rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.pose_cb)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))
        rospy.Timer(rospy.Duration(4.0), self.update_target)
        rospy.Timer(rospy.Duration(8.0), self.check_stuck)
        rospy.loginfo("✅ Smart Explorer 啟動（減少轉彎版）")
        rospy.spin()

    def pose_cb(self, msg):
        self.robot_pos = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        self.robot_yaw = np.arctan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z))

    def map_cb(self, msg):
        self.map = msg

    def check_stuck(self, event):
        rx, ry = self.robot_pos
        lx, ly = self.last_pos
        dist_moved = np.sqrt((rx-lx)**2 + (ry-ly)**2)
        if dist_moved < 0.05 and self.target is not None:
            rospy.logwarn(f"⚠️ 卡住！放棄目標 {self.target}，移入visited")
            self.visited.append(self.target)
            self.target = None
            self.target_fail_count = 0
        self.last_pos = (rx, ry)

    def get_range(self, ranges, a, inc, d1, d2):
        i1 = max(0, int((np.radians(d1)-a)/inc))
        i2 = max(0, int((np.radians(d2)-a)/inc))
        i1, i2 = min(i1,len(ranges)-1), min(i2,len(ranges)-1)
        vals = [r for r in ranges[i1:i2+1]
                if not np.isnan(r) and not np.isinf(r) and 0.05 < r < 6.0]
        return min(vals) if vals else 9.9

    def update_target(self, event):
        if self.map is None:
            return
        data = np.array(self.map.data).reshape(
            self.map.info.height, self.map.info.width)
        res = self.map.info.resolution
        ox  = self.map.info.origin.position.x
        oy  = self.map.info.origin.position.y
        rx, ry = self.robot_pos

        frontiers = []
        for y in range(2, self.map.info.height-2, 3):
            for x in range(2, self.map.info.width-2, 3):
                if data[y,x] == 0:
                    if -1 in [data[y-1,x],data[y+1,x],data[y,x-1],data[y,x+1]]:
                        wx = ox + x*res
                        wy = oy + y*res
                        if not any(abs(wx-vx)<0.2 and abs(wy-vy)<0.2
                                   for vx,vy in self.visited):
                            frontiers.append((wx,wy))

        if not frontiers:
            rospy.loginfo("🏁 探索完成！")
            self.target = None
            return

        if self.target is not None:
            tx, ty = self.target
            dist_to_cur = np.sqrt((tx-rx)**2+(ty-ry)**2)
            if dist_to_cur > 0.4:
                self.target_fail_count += 1
                if self.target_fail_count >= 3:
                    rospy.logwarn(f"❌ 目標 {self.target} 失敗次數過多，加入黑名單")
                    self.visited.append(self.target)
                    self.target = None
                    self.target_fail_count = 0
                else:
                    return

        # ✅ 選擇 frontier 時加入方向懲罰，優先選前方的點
        def score(f):
            dx = f[0] - rx
            dy = f[1] - ry
            dist = np.sqrt(dx**2 + dy**2)
            angle_to = np.arctan2(dy, dx)
            angle_diff = abs(angle_to - self.robot_yaw)
            while angle_diff > np.pi: angle_diff -= 2*np.pi
            angle_diff = abs(angle_diff)
            return dist + 2.0 * angle_diff  # 轉彎代價加重

        mid = [(f, np.sqrt((f[0]-rx)**2+(f[1]-ry)**2)) for f in frontiers]
        near = [(f,d) for f,d in mid if 0.5 < d < 3.0]
        if not near:
            near = sorted(mid, key=lambda x:x[1])[:5]

        # 依 score 排序，取前3再隨機選1
        near_scored = sorted([f for f,d in near], key=score)
        self.target = random.choice(near_scored[:3])
        self.target_fail_count = 0
        rospy.loginfo(f"🗺️ 新目標:({self.target[0]:.1f},{self.target[1]:.1f}) 共{len(frontiers)}個frontier")

    def scan_cb(self, msg):
        r   = msg.ranges
        a   = msg.angle_min
        inc = msg.angle_increment

        front   = self.get_range(r,a,inc,-30, 30)
        left    = self.get_range(r,a,inc, 50, 90)
        right   = self.get_range(r,a,inc,-90,-50)
        front_l = self.get_range(r,a,inc, 20, 60)
        front_r = self.get_range(r,a,inc,-60,-20)

        cmd = Twist()
        now = rospy.Time.now()

        # 脫困
        if self.escape_start is not None:
            elapsed = (now - self.escape_start).to_sec()
            if self.escape_phase == 0:
                if elapsed < 1.0:
                    cmd.linear.x = -0.08
                    self.pub.publish(cmd); return
                else:
                    self.escape_phase = 1; self.escape_start = now; return
            else:
                if elapsed < 2.0:  # ✅ 脫困轉彎縮短到2秒
                    cmd.angular.z = self.turn_dir * 0.03
                    self.pub.publish(cmd); return
                else:
                    self.escape_start = None; self.escape_phase = 0

        # 卡住脫困
        if (front < self.DIST_WARN) and (left < 0.25 or right < 0.25):
            self.turn_dir = 1.0 if left > right else -1.0
            self.escape_start = now; self.escape_phase = 0
            self.turn_start = None
            cmd.linear.x = -0.08; self.pub.publish(cmd); return

        # 側邊太近
        # 兩側都近 = 窄走道，直走不轉
        if right < self.DIST_SIDE and left < self.DIST_SIDE:
            cmd.linear.x = 0.06
            self.pub.publish(cmd); return
        if right < self.DIST_SIDE:
            cmd.linear.x = 0.06; cmd.angular.z = 0.03
            self.pub.publish(cmd); return
        if left < self.DIST_SIDE:
            cmd.linear.x = 0.06; cmd.angular.z = -0.03
            self.pub.publish(cmd); return

        # 轉彎鎖定
        if self.turn_start is not None:
            if (now - self.turn_start).to_sec() < self.TURN_DUR:
                cmd.angular.z = self.turn_dir * 0.03
                self.pub.publish(cmd); return
            self.turn_start = None

        # 前方障礙
        if front < self.DIST_WARN:
            self.turn_dir = -1.0 if front_l > front_r else 1.0
            self.turn_start = now
            self.TURN_DUR = random.uniform(2.0, 3.0)  # ✅ 縮短轉彎時間
            cmd.angular.z = self.turn_dir * 0.03
            self.pub.publish(cmd); return

        # 朝 frontier 走
        if self.target is not None:
            rx, ry = self.robot_pos
            tx, ty = self.target
            dist = np.sqrt((tx-rx)**2+(ty-ry)**2)
            if dist < 0.3:
                self.visited.append(self.target)
                self.target = None
                self.target_fail_count = 0
            else:
                target_yaw = np.arctan2(ty-ry, tx-rx)
                angle_err  = target_yaw - self.robot_yaw
                while angle_err >  np.pi: angle_err -= 2*np.pi
                while angle_err < -np.pi: angle_err += 2*np.pi

                if abs(angle_err) > 0.3:  # ✅ 降低門檻：17度就開始修正（原本57度）
                    # ✅ 邊走邊轉，不再原地旋轉
                    cmd.linear.x  = self.FWD_SPEED * 0.5
                    cmd.angular.z = np.sign(angle_err) * 0.04
                else:
                    cmd.linear.x  = self.FWD_SPEED
                    cmd.angular.z = max(-0.04, min(0.04, angle_err * 0.15))
        else:
            cmd.linear.x = self.FWD_SPEED

        self.pub.publish(cmd)

SmartExplorer()
