import { forwardRef } from "react";
import { View } from "react-native";
import { Text } from "../ui/text";

type StepStatus = "completed" | "in-progress" | "not-started" | "failed";

interface TaskStepProps {
  title: string;
  status: StepStatus;
}

const TaskStep = forwardRef<View, TaskStepProps>(({ title, status }, ref) => {
  const badgeColor: Partial<Record<StepStatus, string>> = {
    "in-progress": "",
    "not-started": "opacity-30",
    failed: "bg-red-500",
  };

  const textOpacity: Partial<Record<StepStatus, string>> = {
    "not-started": "opacity-40",
    failed: "text-red-500",
  };

  return (
    <View
      ref={ref}
      className={`flex-row items-center gap-2 px-2 py-2 ${
        status === "in-progress" &&
        "rounded-md border border-outline-200 bg-primary-400/5"
      } ${status === "failed" && "rounded-md border border-red-200 bg-red-50"}`}
    >
      <View
        className={`size-2 rounded-full bg-primary-500 ${badgeColor[status]}`}
      />
      <Text className={`${textOpacity[status]}`}>{title}</Text>
    </View>
  );
});

export default TaskStep;
