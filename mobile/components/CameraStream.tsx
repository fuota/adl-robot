import { useRos } from "@/contexts/RosContext";
import { useEffect, useState } from "react";
import { Image, Text, View } from "react-native";

export default function CameraStream() {
  const { ros, connected } = useRos();
  const [imageData, setImageData] = useState<string>();

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

  if (!connected) {
    return (
      <View className="w-full items-center justify-center bg-gray-100" style={{ aspectRatio: 4/3 }}>
        <Text className="text-gray-500">Connecting to ROS...</Text>
      </View>
    );
  }

  return imageData ? (
    <Image 
      source={{ uri: imageData }} 
      className="w-full" 
      style={{ aspectRatio: 4/3 }}
      resizeMode="contain"
    />
  ) : (
    <View className="w-full items-center justify-center bg-black" style={{ aspectRatio: 4/3 }}>
      <Text className="text-white">Waiting for camera feed...</Text>
    </View>
  );
}
