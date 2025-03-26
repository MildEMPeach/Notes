# ROS2
## workspace


## 话题通信
Topic: 话题（发送信息的管道）ros2 topic -h
Interface: 接口（规定管道中发送信息的格式）ros2 interface -h
Node: 结点 ros2 node 

Node既可以订阅话题，也可以发布话题。通过Node订阅的话题可以向Node传递控制信息，通过Node发布的话题可以了解Node的当前状态。

### self-define node to finish publish and subscribe
**python version:**
```python
# Location: workspace/src/demo_python_pkg/demo_node.py
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class demo_node(Node):
    def __init__(self):
        super().__init__("demo_node")
        self.item = String()
        self.item.data = 'This is a msg'
        # To publish something
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        """
        The first arg should be the type of interface,
        the second arg should be the name of the topic,
        the third arg should be qos, type=int, and not that important.
        """
        # To subscribe something
        self.received_data = ""
        self.subscribe = self.create_subscription(
            String,
            'another_topic_name', 
            self.call_back, 
            10
        )
        """
        The first arg should be the type of the interface/message the node will receive,
        the second arg should be the name of the topic the node subscribed,
        the third arg should be the function the node will use to get the message,
        the fourth arg should be qos, type=int, and not that important.
        """

    def call_back(self, msg):
        # msg will be resumed as String
        self.received_data = msg.data        
    
    def publish(self):
        self.publisher.publish(self.item) # to publish something here
    
    """
    Some Other functions here
    """

def main():
    rclpy.init()
    node = demo_node()
    """
    Some code here
    """
    rclpy.spin(node)
    rclpy.shutdown()

```
