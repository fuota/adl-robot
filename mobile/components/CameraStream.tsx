import { useRos } from "@/contexts/RosContext";
import { useEffect, useState } from "react";
import { Image, Text, View } from "react-native";

type ObjectMap = {
  [key: string]: {
    name: string;
    x: number; // 0-1 range (0=left, 1=right)
    y: number; // 0-1 range (0=top, 1=bottom)
  };
};

type Props = {
  showObjects?: boolean;
};

export default function CameraStream({ showObjects = false }: Props) {
  const { ros, connected } = useRos();
  const [imageData, setImageData] = useState<string>();
  const [objectMap, setObjectMap] = useState<ObjectMap>({});

  // Format object name: remove underscores, capitalize first letter of each word
  const formatObjectName = (name: string): string => {
    return name
      .split("_")
      .map((word) => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
      .join(" ");
  };

  // Subscribe to camera feed
  useEffect(() => {
    if (!connected) {
      setImageData(undefined);
      return;
    }

    // Dynamically require ROSLIB to avoid bundling it into native builds.
    // eslint-disable-next-line @typescript-eslint/no-var-requires
    const ROSLIB = require("roslib");

    ros.on("connection", () => {
      console.log("Connected to ROS bridge");
    });

    const imageTopic = new ROSLIB.Topic({
      ros,
      name: "/camera/camera/color/image_raw/compressed",
      messageType: "sensor_msgs/msg/CompressedImage",
    });

    imageTopic.subscribe((message: any) => {
      setImageData(`data:image/jpeg;base64,${message.data}`);
    });

    return () => {
      try {
        imageTopic.unsubscribe();
      } catch (e) {
        // ignore
      }
    };
  }, [ros, connected]);

  // Subscribe to detected objects
  useEffect(() => {
    if (!connected || !showObjects) {
      setObjectMap({});
      return;
    }

    try {
      // eslint-disable-next-line @typescript-eslint/no-var-requires
      const ROSLIB = require("roslib");

      const objectMapTopic = new ROSLIB.Topic({
        ros,
        name: "/detected_objects/object_map",
        messageType: "std_msgs/String",
      });

      objectMapTopic.subscribe((message: any) => {
        try {
          const parsed = JSON.parse(message.data);
          setObjectMap(parsed);
          console.log("[CameraStream] Received object map:", parsed);
        } catch (e) {
          console.error("[CameraStream] Error parsing object map:", e);
        }
      });

      return () => {
        try {
          objectMapTopic.unsubscribe();
        } catch (e) {
          // ignore
        }
      };
    } catch (e) {
      console.error("[CameraStream] Error subscribing to object map:", e);
    }
  }, [ros, connected, showObjects]);

  if (!connected) {
    return (
      <View className="w-full items-center justify-center bg-gray-100" style={{ aspectRatio: 4/3 }}>
        <Text className="text-gray-500">Connecting to ROS...</Text>
      </View>
    );
  }

  return imageData ? (
    <View className="w-full relative" style={{ aspectRatio: 4/3 }}>
      <Image 
        source={{ uri: imageData }} 
        className="w-full h-full" 
        style={{ aspectRatio: 4/3 }}
        resizeMode="contain"
      />
      {showObjects && (
        <View className="absolute inset-0" pointerEvents="none">
          {Object.entries(objectMap).map(([id, obj]) => {
            // Position label above the ArUco marker (2.5cm x 2.5cm)
            // x and y are center coordinates in 0-1 range
            // Position label above the marker center
            const labelOffset = -25; // Offset in pixels to position above marker
            
            return (
              <View
                key={id}
                style={{
                  position: "absolute",
                  left: `${obj.x * 100}%`,
                  top: `${obj.y * 100}%`,
                  transform: [{ translateX: -50 }, { translateY: labelOffset }], // Center horizontally, position above marker
                }}
              >
                <View
                  style={{
                    backgroundColor: "rgba(0, 0, 0, 0.85)",
                    paddingHorizontal: 10,
                    paddingVertical: 6,
                    borderRadius: 6,
                    borderWidth: 2,
                    borderColor: "rgba(255, 255, 255, 0.5)",
                    shadowColor: "#000",
                    shadowOffset: { width: 0, height: 2 },
                    shadowOpacity: 0.5,
                    shadowRadius: 3,
                    elevation: 5, // For Android
                  }}
                >
                  <Text
                    style={{
                      color: "#FFFFFF",
                      fontSize: 13,
                      fontWeight: "bold",
                      textShadowColor: "rgba(0, 0, 0, 0.8)",
                      textShadowOffset: { width: 1, height: 1 },
                      textShadowRadius: 3,
                      letterSpacing: 0.5,
                    }}
                  >
                    {formatObjectName(obj.name)}
                  </Text>
                </View>
              </View>
            );
          })}
        </View>
      )}
    </View>
  ) : (
    <View className="w-full items-center justify-center bg-black" style={{ aspectRatio: 4/3 }}>
      <Text className="text-white">Waiting for camera feed...</Text>
    </View>
  );
}
