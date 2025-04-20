import rospy
from geometry_msgs.msg import Point
import cv2
import mediapipe as mp

class PersonFollower:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('person_follower', anonymous=True)
        
        # 创建发布器，发布鼻子位置
        self.pub_nose_position = rospy.Publisher('/nose_position', Point, queue_size=10)

        # 初始化MediaPipe Holistic模块，使用低复杂度和低置信度设置以优化性能
        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(min_detection_confidence=0.3, min_tracking_confidence=0.3, model_complexity=0)
        self.mp_drawing = mp.solutions.drawing_utils
        self.window_name = 'MediaPipe Holistic'

        # 创建显示窗口
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def run(self):
        # 创建视频捕捉对象。其参数0表示第一个摄像头
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("错误：无法打开摄像头")
            return

        try:
            while not rospy.is_shutdown():
                # 从摄像头读取一帧
                ret, frame = cap.read()
                if not ret:
                    print("错误：无法从摄像头读取数据")
                    break

                # 使用MediaPipe进行人物检测
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.holistic.process(image)
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                # 绘制检测到的关键点
                if results.pose_landmarks:
                    self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS)

                    # 获取鼻子的位置并发布
                    nose = results.pose_landmarks.landmark[self.mp_holistic.PoseLandmark.NOSE]
                    image_width, image_height = image.shape[1], image.shape[0]
                    nose_position = Point()
                    nose_position.x = nose.x * image_width - image_width / 2
                    nose_position.y = nose.y * image_height - image_height / 2
                    self.pub_nose_position.publish(nose_position)

                # 显示图像窗口
                cv2.imshow(self.window_name, image)

                # 按'q'键退出循环
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            # 释放摄像头资源
            cap.release()
            # 关闭所有OpenCV窗口
            cv2.destroyAllWindows()

if __name__ == '__main__':
    follower = PersonFollower()
    follower.run()