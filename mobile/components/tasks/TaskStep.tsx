import { forwardRef } from "react";
import { View, ActivityIndicator } from "react-native";
import { Text } from "../ui/text";
import { CheckCircle2, XCircle, Circle } from "lucide-react-native";

type StepStatus = "completed" | "in-progress" | "not-started" | "failed";

interface TaskStepProps {
  title: string;
  status: StepStatus;
}

const TaskStep = forwardRef<View, TaskStepProps>(({ title, status }, ref) => {
  // Render icon based on status
  const renderIcon = () => {
    switch (status) {
      case "completed":
        return <CheckCircle2 size={20} color="#10b981" />; // green check
      case "in-progress":
        return (
          <View className="items-center justify-center">
            <ActivityIndicator size="small" color="#3b82f6" />
          </View>
        ); // blue spinner
      case "failed":
        return <XCircle size={20} color="#ef4444" />; // red X
      case "not-started":
      default:
        return <Circle size={20} color="#9ca3af" />; // gray circle
    }
  };

  const textColor: Partial<Record<StepStatus, string>> = {
    "not-started": "text-gray-500",
    "in-progress": "text-blue-600 font-semibold",
    completed: "text-green-600",
    failed: "text-red-600 font-semibold",
  };

  const backgroundColor: Partial<Record<StepStatus, string>> = {
    "in-progress": "bg-blue-50 border-blue-200",
    failed: "bg-red-50 border-red-200",
    completed: "bg-green-50 border-green-200",
    "not-started": "",
  };

  return (
    <View
      ref={ref}
      className={`flex-row items-center gap-3 px-3 py-2 rounded-md ${
        status === "in-progress" && "border-2"
      } ${status === "failed" && "border-2"} ${
        status === "completed" && "border-2"
      } ${backgroundColor[status] || ""}`}
    >
      {renderIcon()}
      <Text className={`flex-1 ${textColor[status] || "text-gray-700"}`}>
        {title}
      </Text>
    </View>
  );
});

export default TaskStep;
