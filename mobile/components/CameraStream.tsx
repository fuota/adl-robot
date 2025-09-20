import { useRos } from "@/contexts/RosContext";
import { useEffect, useState } from "react";
import { Image } from "react-native";
import ROSLIB from "roslib";

export default function CameraStream() {
  const { ros } = useRos();
  const [imageData, setImageData] = useState<string>();

  useEffect(() => {
    ros.on("connection", () => {
      console.log("Connected to ROS bridge");
    });

    const imageTopic = new ROSLIB.Topic({
      ros,
      name: "/camera/camera/color/image_raw/compressed",
      messageType: "sensor_msgs/msg/CompressedImage",
    });

    imageTopic.subscribe((message) => {
      // @ts-expect-error ROSLIB.Message type definitions are incomplete
      setImageData(`data:image/jpeg;base64,${message.data}`);
    });

    return () => {
      imageTopic.unsubscribe();
    };
  }, []);

  return (
    imageData && <Image source={{ uri: imageData }} className="h-full w-full" />
  );
}
