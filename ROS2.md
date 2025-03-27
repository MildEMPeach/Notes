# ROS2
## Learning Resources:
[fishros-bilibili](https://www.bilibili.com/video/BV1GW42197Ck/?spm_id_from=333.1387.collection.video_card.click)
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

**cpp version**
```cpp
// Use turtlesimnode as a demo.
// We will define a node to control the turtle. 
// Firstly, we get its postion by subscribption. 
// Secondly, we change its position by publish.
// Location: workspace/src/demo_cpp_pkg/src/turtle_control.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

class TurtleControlNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription;

    double target_x{1.0};
    double target_y{1.0};
    double k{1.0};
    double max_speed{3.0};

public:
    TurtleControlNode() : Node("turtle_control")
    {
        this->publisher = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        this->subscription = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleControlNode::pose_callback, this, std::placeholders::_1));
    };

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {   
        auto current_x = msg->x;
        auto current_y = msg->y;
        RCLCPP_INFO(this->get_logger(), "Current position: x=%.2f, y=%.2f", current_x, current_y);
        
        auto distance = std::sqrt((target_x - current_x) * (target_x - current_x) + (target_y - current_y) * (target_y - current_y));
        auto angle = std::atan2(target_y - current_y, target_x - current_x) - msg->theta;
        
        auto control_msg = geometry_msgs::msg::Twist();
        if (distance > 0.1) {
            if (fabs(angle) > 0.2) {
                control_msg.angular.z = fabs(angle);   
            } else {
                control_msg.linear.x = k * distance > max_speed ? max_speed : k * distance;
            }
        }

        this->publisher->publish(control_msg);
    };

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

```
