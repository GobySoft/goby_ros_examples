# goby_ros_examples

## JSON Example

The `example_json.cpp` (built into `libgoby_ros_examples_json`) provides an implementation of `goby::ros::TranslatorToROS` and `goby::ros::TranslatorToGoby` for JSON messages:

- Goby -> ROS: Goby group: `json_to_ros`, ROS Topic `json_from_goby`
- ROS -> Goby: Goby group: `json_from_ros`, ROS Topic `json_to_goby`

The message formats used are:

- Goby: nlohmann::json (marshalling scheme JSON = 7)
- ROS: std_msgs::msg::String

### Running

At its simplest you just need to run:

- `gobyd`
- `goby_ros_gateway`

We will also use the `goby zeromq [publish/subscribe]` and `ros2 topic [pub/echo]` command line tools to publish and subscribe data from both sides of the gateway.

The core apps and the publishers (both ROS and Goby) are launched via a `goby_launch` script (you could instead easily use a ROS 2 launch file if you prefer):

```
cd goby_ros_examples/launch
./goby_ros_json.launch
```

Then to see the published data, you can manually run either or both of the subscriber/echo tools:

```
> goby zeromq subscribe json_from_ros
7 | json_from_ros | nlohmann::json | 2024-Oct-23 18:48:25.805422 | {"a":10,"b":9,"c":8}
7 | json_from_ros | nlohmann::json | 2024-Oct-23 18:48:25.905056 | {"a":10,"b":9,"c":8}
7 | json_from_ros | nlohmann::json | 2024-Oct-23 18:48:26.904825 | {"a":10,"b":9,"c":8}

> ros2 topic echo /json_from_goby
data: '{"x":3,"y":4,"z":5}'
---
data: '{"x":3,"y":4,"z":5}'
---
data: '{"x":3,"y":4,"z":5}'
---
```

If you are using JSON in your work you can copy example_json.cpp as a starting point to create a custom plugin with the desired Goby group(s) and ROS 2 topic(s).

### Expanding on this example: ROS message as JSON

We can use `rosidl_runtime_py` to convert a ROS Msg into a JSON string suitable for passing through this `goby_ros_gateway` plugin.

In `goby_ros_examples/msg/Nav.msg` we define a simple Msg:

```
float64 lat
float64 lon
```

and in `goby_ros_examples/scripts/nav_json_publisher.py` we publish it as a JSON string (in std_msgs::msg::String).

To run:

```
ros2 run goby_ros_examples nav_json_publisher.py
```

and now we see both the original publication and this new ROS Msg publication on the Goby side.

```
> goby zeromq subscribe json_from_ros
7 | json_from_ros | nlohmann::json | 2024-Oct-23 19:27:01.069215 | {"lat":47.5,"lon":112.3}
7 | json_from_ros | nlohmann::json | 2024-Oct-23 19:27:01.569330 | {"lat":47.5,"lon":112.3}
7 | json_from_ros | nlohmann::json | 2024-Oct-23 19:27:01.793890 | {"a":10,"b":9,"c":8}
7 | json_from_ros | nlohmann::json | 2024-Oct-23 19:27:02.069403 | {"lat":47.5,"lon":112.3}
7 | json_from_ros | nlohmann::json | 2024-Oct-23 19:27:02.569230 | {"lat":47.5,"lon":112.3}
7 | json_from_ros | nlohmann::json | 2024-Oct-23 19:27:02.794293 | {"a":10,"b":9,"c":8}
7 | json_from_ros | nlohmann::json | 2024-Oct-23 19:27:03.069126 | {"lat":47.5,"lon":112.3}
```

## Protobuf Example

Google Protocol Buffers ("Protobuf") is commonly used as the marshalling scheme in Goby3. While ROS Msg provides much of the same functionality as Protobuf, there's no standard or obvious way to convert between the two. 

The example in  `example_protobuf.cpp` provides an implementation of `goby::ros::TranslatorToROS` and `goby::ros::TranslatorToGoby` for a given Protobuf message (nav.proto):

- Goby -> ROS: Goby group: `protobuf_to_ros`, ROS Topic `protobuf_from_goby`
- ROS -> Goby: Goby group: `protobuf_from_ros`, ROS Topic `protobuf_to_goby`

The message formats used are:

- Goby: goby_ros_examples::protobuf::Nav (marshalling scheme PROTOBUF = 1)
- ROS: std_msgs::msg::String (for both TextFormat Protobuf encoding and Base64 binary encoding) and goby_ros_examples Nav msg (for direct copy between Protobuf and equivalent ROS Msg).

### Running

```
cd goby_ros_examples/launch
./goby_ros_protobuf.launch
```

To see the published data, you can run:

```
> goby zeromq subscribe protobuf_from_ros

> ros2 topic echo /protobuf_from_goby
data: 'lat: 42.5 lon: -12.3'
---
data: 'lat: 42.5 lon: -12.3'
---
data: 'lat: 42.5 lon: -12.3'
---
> ros2 topic echo /nav_encoded_from_goby
data: CQAAAAAAQEVAEZqZmZmZmSjA
---
data: CQAAAAAAQEVAEZqZmZmZmSjA
---
data: CQAAAAAAQEVAEZqZmZmZmSjA
> ros2 topic echo /nav_from_goby 

lat: 42.5
lon: -12.3
---
lat: 42.5
lon: -12.3
---
lat: 42.5
lon: -12.3

```

You can check the encoding on the command line using:

```
echo "CQAAAAAAQEVAEZqZmZmZmSjA" | base64 --decode | protoc nav.proto --decode=goby_ros_examples.protobuf.Nav
```
