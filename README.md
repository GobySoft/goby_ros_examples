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

The example in  `example_protobuf.cpp` shows three ways one could interoperate between Goby3 when using Protobuf and ROS:

1. JSON encoding of Protobuf.
2. Binary encoding of Protobuf.
3. Conversion between equivalent Protobuf and ROS Msg by copying fields directly.

The example provides an implementation of `goby::ros::TranslatorToROS` and `goby::ros::TranslatorToGoby` for a given Protobuf message (nav.proto).

### JSON Option

Groups/Topics:
- Goby -> ROS: Goby group: `protobuf_to_ros`, ROS Topic `protobuf_json_from_goby`
- ROS -> Goby: Goby group: `protobuf_from_ros`, ROS Topic `protobuf_json_to_goby`

The message formats used are:

- Goby: goby_ros_examples::protobuf::Nav (marshalling scheme PROTOBUF = 1)
- ROS: goby_ros_examples::msg::ProtobufJSON

Advantages: 
- Human readable, JSON is widely supported message format
- Could use Protobuf message parser or directly read JSON message

Disadvantages:
- Encoding speed is slower (JSON is string object)
- Message size is larger (JSON versus Protobuf encoding)

### Encoded Option

Groups/Topics:
- Goby -> ROS: Goby group: `protobuf_to_ros`, ROS Topic `protobuf_encoded_from_goby`
- ROS -> Goby: Goby group: `protobuf_from_ros`, ROS Topic `protobuf_encoded_to_goby`

The message formats used are:

- Goby: goby_ros_examples::protobuf::Nav (marshalling scheme PROTOBUF = 1)
- ROS: goby_ros_examples::msg::ProtobufEncoded

Advantages:
- Fast (encoding time) and efficient (message size)

Disadvantages:
- ROS Node must have Protobuf message to decode
- Not human readable

### Copy between equivalent messages option

Groups/Topics:
- Goby -> ROS: Goby group: `protobuf_to_ros`, ROS Topic `nav_from_goby`
- ROS -> Goby: Goby group: `protobuf_from_ros`, ROS Topic `nav_to_goby`

The message formats used are:

- Goby: goby_ros_examples::protobuf::Nav (marshalling scheme PROTOBUF = 1)
- ROS: goby_ros_examples::msg::Nav (equivalent ROS msg to goby_ros_examples::protobuf::Nav).

Advantages:
- Native ROS Msg used on ROS side.

Disadvantages:
- Copying between Protobuf and ROS Msg must be done manually; ROS Msg and copying code must be kept "in sync" with Protobuf message definition.

### Running

```
cd goby_ros_examples/launch
./goby_ros_protobuf.launch
```

To see the published data, you can run:

```
> goby zeromq subscribe protobuf_from_ros -l ../../../build/goby_ros_examples/libgoby_ros_examples_protobuf.so
# from JSON publish
1 | protobuf_from_ros | goby_ros_examples.protobuf.Nav | 2024-Oct-24 20:36:14.336038 | lat: 12 lon: -15.3
# from nav publish
1 | protobuf_from_ros | goby_ros_examples.protobuf.Nav | 2024-Oct-24 20:36:14.336956 | lat: 42.5 lon: -12.3
# from encoded publisher (ros_protobuf_encoded_publisher node)
1 | protobuf_from_ros | goby_ros_examples.protobuf.Nav | 2024-Oct-24 20:36:14.337759 | lat: 100.3 lon: 34.2


> ros2 topic echo /protobuf_json_from_goby
pb_type: goby_ros_examples.protobuf.Nav
json: '{"lat":42.5,"lon":-12.3}'
---
pb_type: goby_ros_examples.protobuf.Nav
json: '{"lat":42.5,"lon":-12.3}'
---
pb_type: goby_ros_examples.protobuf.Nav
json: '{"lat":42.5,"lon":-12.3}'

> ros2 run goby_ros_examples ros_protobuf_encoded_subscriber
[INFO] [1729802680.790605157] [protobuf_subscriber]: Rx (goby_ros_examples.protobuf.Nav): 'lat: 42.5 lon: -12.3'
[INFO] [1729802682.790822361] [protobuf_subscriber]: Rx (goby_ros_examples.protobuf.Nav): 'lat: 42.5 lon: -12.3'
[INFO] [1729802684.789645028] [protobuf_subscriber]: Rx (goby_ros_examples.protobuf.Nav): 'lat: 42.5 lon: -12.3'

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
