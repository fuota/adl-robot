import { useRos } from "@/contexts/RosContext";
import { useEffect, useState } from "react";
import { Image, Platform, Text, View } from "react-native";

export default function CameraStream() {
  // Immediately return a native-friendly placeholder so Expo Go on iPad
  // renders the UI without attempting any ROS/network work.
  if (Platform.OS !== "web") {
    return (
      <View className="h-full w-full items-center justify-center bg-gray-100">
        <Text className="text-gray-500">Camera disabled for demo (ROS disabled on native)</Text>
      </View>
    );
  }

  // Web-only path: use ROS normally
  const { ros } = useRos();
  const [imageData, setImageData] = useState<string>();

  useEffect(() => {
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
  }, [ros]);

  return imageData ? (
    <Image source={{ uri: imageData }} className="h-full w-full" />
  ) : (
    <View className="h-full w-full bg-black" />
  );
}
