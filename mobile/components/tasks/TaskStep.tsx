import React, { forwardRef } from "react";
import { View } from "react-native";
import { Text } from "../ui/text";

type StepStatus = "completed" | "in-progress" | "not-started";

interface TaskStepProps {
  title: string;
  status: StepStatus;
}

const TaskStep = forwardRef<View, TaskStepProps>(
  ({ title, status }, ref) => {
    const badgeColor: Partial<Record<StepStatus, string>> = {
      "in-progress": "",
      "not-started": "opacity-30",
    };

    return (
      <View
        ref={ref}
        className={`flex-row items-center gap-2 px-2 py-2 ${
          status === "in-progress" &&
          "rounded-md border border-outline-200 bg-primary-400/5"
        }`}
      >
        <View
          className={`size-2 rounded-full bg-primary-500 ${badgeColor[status]}`}
        />
        <Text className={`${status === "not-started" && "opacity-40"}`}>
          {title}
        </Text>
      </View>
    );
  }
);

export default TaskStep;
