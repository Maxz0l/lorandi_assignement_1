#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import math


class TablePublisher(Node):
    """
    Node qui reçoit les détections de tables et publie les 3 positions finales.
    """
    
    def __init__(self):
        super().__init__('table_publisher')
        
        # Subscriber aux détections de tables
        self.subscription = self.create_subscription(
            PoseArray,
            '/detected_tables',
            self.tables_callback,
            10
        )
        
        # Publisher des 3 tables finales
        self.tables_pub = self.create_publisher(PoseArray, '/final_tables', 10)
        
        # Stockage des tables détectées avec historique
        self.table_history = []
        self.history_max_size = 50
        
        # Timer pour publier périodiquement
        self.pub_timer = self.create_timer(1.0, self.publish_final_tables)
        
        self.get_logger().info('TablePublisher node initialized.')
    
    def tables_callback(self, msg):
        """Callback qui reçoit les détections de tables"""
        
        # Ajouter les détections à l'historique
        if len(msg.poses) > 0:
            self.table_history.append(msg.poses)
            
            # Limiter la taille de l'historique
            if len(self.table_history) > self.history_max_size:
                self.table_history.pop(0)
            
            self.get_logger().info(f'Received {len(msg.poses)} table detections')
    
    def publish_final_tables(self):
        """Publie les 3 meilleures positions de tables"""
        
        if len(self.table_history) == 0:
            return
        
        # Clustering : regrouper les détections proches
        all_tables = []
        for detection in self.table_history:
            all_tables.extend(detection)
        
        if len(all_tables) == 0:
            return
        
        # Regrouper les tables par proximité (clustering simple)
        clusters = self.cluster_tables(all_tables)
        
        # Garder les 3 clusters les plus robustes
        if len(clusters) >= 3:
            final_tables = clusters[:3]
        else:
            self.get_logger().warn(f'Only {len(clusters)} table clusters found (need 3)')
            final_tables = clusters
        
        # Créer le message de sortie
        output = PoseArray()
        output.header.frame_id = 'odom'
        output.header.stamp = self.get_clock().now().to_msg()
        output.poses = final_tables
        
        self.tables_pub.publish(output)
        
        self.get_logger().info(f'Published {len(final_tables)} final table positions:')
        for i, pose in enumerate(final_tables):
            self.get_logger().info(f'  Table {i+1}: x={pose.position.x:.2f}, y={pose.position.y:.2f}')
    
    def cluster_tables(self, tables, distance_threshold=0.5):
        """
        Regroupe les détections de tables proches (clustering simple).
        Retourne les centres des clusters.
        """
        
        if len(tables) == 0:
            return []
        
        clusters = []
        used = [False] * len(tables)
        
        for i, table in enumerate(tables):
            if used[i]:
                continue
            
            # Nouveau cluster
            cluster_tables = [table]
            used[i] = True
            
            # Trouver toutes les tables proches
            for j, other_table in enumerate(tables):
                if used[j]:
                    continue
                
                dist = math.sqrt(
                    (table.position.x - other_table.position.x)**2 +
                    (table.position.y - other_table.position.y)**2
                )
                
                if dist < distance_threshold:
                    cluster_tables.append(other_table)
                    used[j] = True
            
            # Calculer le centre du cluster (moyenne)
            avg_x = sum(t.position.x for t in cluster_tables) / len(cluster_tables)
            avg_y = sum(t.position.y for t in cluster_tables) / len(cluster_tables)
            avg_z = sum(t.position.z for t in cluster_tables) / len(cluster_tables)
            
            center = Pose()
            center.position.x = avg_x
            center.position.y = avg_y
            center.position.z = avg_z
            center.orientation.w = 1.0
            
            clusters.append(center)
        
        # Trier par nombre de détections (les plus robustes d'abord)
        # (ici on ne trie pas, mais on pourrait stocker le nombre de détections par cluster)
        
        return clusters


def main(args=None):
    rclpy.init(args=args)
    node = TablePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()