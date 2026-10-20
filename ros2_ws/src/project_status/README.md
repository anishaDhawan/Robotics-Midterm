# project_status

A ROS 2 Humble package that defines the custom `Status` message type for tracking robot mission state, emergency conditions, and human detection.

---

## Message: `project_status/msg/Status`

| Field | Type | Description |
|---|---|---|
| `goal1_received` | `bool` | Whether Goal 1 has been received |
| `goal1_achieved` | `bool` | Whether Goal 1 has been achieved |
| `goal2_received` | `bool` | Whether Goal 2 has been received |
| `goal2_achieved` | `bool` | Whether Goal 2 has been achieved |
| `emergency` | `bool` | `false` = no emergency, `true` = handling emergency |
| `human_found` | `bool` | `false` = not found, `true` = human detected |
| `human_location` | `geometry_msgs/Point` | Human position in `/map` frame — ignore `z` |

---

## Build Instructions

Place this package inside your ROS 2 workspace `src/` directory, then build:

```bash
# 1. Navigate to your workspace root
cd ~/ros2_ws

# 2. Copy (or clone) the package
cp -r /path/to/project_status src/

# 3. Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --packages-select project_status

# 5. Source the workspace
source install/setup.bash
```

---

## Verify the Message

```bash
ros2 interface show project_status/msg/Status
```

Expected output:
```
bool goal1_received
bool goal1_achieved
bool goal2_received
bool goal2_achieved
bool emergency
bool human_found
geometry_msgs/Point human_location
        float64 x
        float64 y
        float64 z
```

---

## Usage Examples

### Python Publisher

```python
import rclpy
from rclpy.node import Node
from project_status.msg import Status
from geometry_msgs.msg import Point

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.pub = self.create_publisher(Status, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        msg = Status()
        msg.goal1_received = True
        msg.goal1_achieved = False
        msg.goal2_received = False
        msg.goal2_achieved = False
        msg.emergency = False
        msg.human_found = True
        msg.human_location = Point(x=1.5, y=2.3, z=0.0)  # z is ignored
        self.pub.publish(msg)
        self.get_logger().info('Published status')

def main():
    rclpy.init()
    node = StatusPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```

### C++ Publisher

```cpp
#include "rclcpp/rclcpp.hpp"
#include "project_status/msg/status.hpp"

class StatusPublisher : public rclcpp::Node {
public:
    StatusPublisher() : Node("status_publisher") {
        pub_ = create_publisher<project_status::msg::Status>("robot_status", 10);
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                auto msg = project_status::msg::Status();
                msg.goal1_received = true;
                msg.goal1_achieved = false;
                msg.goal2_received = false;
                msg.goal2_achieved = false;
                msg.emergency = false;
                msg.human_found = true;
                msg.human_location.x = 1.5;
                msg.human_location.y = 2.3;
                msg.human_location.z = 0.0;  // ignored
                pub_->publish(msg);
            });
    }
private:
    rclcpp::Publisher<project_status::msg::Status>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusPublisher>());
    rclcpp::shutdown();
}
```

### Quick Test via CLI

```bash
# Publish a one-shot message
ros2 topic pub /robot_status project_status/msg/Status \
  "{goal1_received: true, goal1_achieved: false, \
    goal2_received: false, goal2_achieved: false, \
    emergency: false, human_found: true, \
    human_location: {x: 1.5, y: 2.3, z: 0.0}}"

# Subscribe and echo
ros2 topic echo /robot_status
```
