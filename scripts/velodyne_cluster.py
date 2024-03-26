#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sklearn.cluster import DBSCAN
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

class SCANCluster:
    def __init__(self):
        # 초기화 메소드: ROS 구독자와 발행자 설정
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.cloud_pub = rospy.Publisher("cluster_centers", PointCloud2, queue_size=10)

        # DBSCAN 파라미터 설정
        self.dbscan = DBSCAN(eps=0.5, min_samples=5)

    def callback(self, msg):
        # 포인트 클라우드 데이터가 도착했을 때 호출되는 콜백 메소드
        cluster_centers = []
        self.pc_np = self.pointcloud2_to_xyz(msg)

        if len(self.pc_np) > 0:
            pc_xy = self.pc_np[:, :2]
            db = self.dbscan.fit_predict(pc_xy)
            n_cluster = np.max(db) + 1

            for c in range(n_cluster):
                c_tmp = np.mean(pc_xy[db == c, :], axis=0)
                cluster_centers.append([c_tmp[0], c_tmp[1], 0])  # Z 값은 0으로 설정

            self.publish_cluster(cluster_centers)

    def pointcloud2_to_xyz(self, cloud_msg):
        # 포인트 클라우드 데이터로부터 XYZ 좌표를 추출하는 메소드
        point_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            
            if 5 > point[0] > 0 and 0.1 > point[2] > -0.6:
                point_list.append((point[0], point[1], point[2], point[3], dist, np.arctan2(point[1], point[0])))

        return np.array(point_list, np.float32)
    
    def publish_cluster(self, cluster_centers):
        # 계산된 클러스터 중심점을 PointCloud2 메시지로 발행하는 메소드
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]
        
        cloud = pc2.create_cloud_xyz32(header, cluster_centers)
        self.cloud_pub.publish(cloud)

if __name__ == '__main__':
    rospy.init_node('velodyne_clustering', anonymous=True)
    scan_cluster = SCANCluster()
    rospy.spin()
