import CameraStream from "@/components/CameraStream";
import TaskStep from "@/components/tasks/TaskStep";
import { Badge, BadgeText } from "@/components/ui/badge";
import { Button, ButtonIcon, ButtonText } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Heading } from "@/components/ui/heading";
import { Progress, ProgressFilledTrack } from "@/components/ui/progress";
import { Text } from "@/components/ui/text";
import { useTask } from "@/contexts/TaskContext";
import {
  ActivityIcon,
  MicIcon,
  PauseIcon,
  PlayIcon,
  RadioIcon,
  SlidersHorizontalIcon,
  SquareIcon,
} from "lucide-react-native";
import { FlatList, View } from "react-native";
import { SafeAreaView } from "react-native-safe-area-context";

export default function TaskScreen() {
  // Read selected id from TaskContext
  const { selectedId, tasks, startTask, currentProgress } = useTask();
  let id = selectedId || "1";
  if (!id && typeof window !== "undefined") {
    id = new URLSearchParams(window.location.search).get("id") ?? "1";
  }

  const task = tasks.find((t) => t.id === id) ?? tasks[0];

  // If no task found, show loading or error
  if (!task) {
    return (
      <SafeAreaView className="flex-1 items-center justify-center bg-gray-50">
        <Text>Task not found</Text>
      </SafeAreaView>
    );
  }

  const handleStartTask = () => {
    startTask(task.id);
  };

  // Determine badge status and color
  const getBadgeProps = () => {
    switch (task.status) {
      case "in-progress":
        return { action: "warning" as const, text: "Running" };
      case "completed":
        return { action: "success" as const, text: "Completed" };
      case "failed":
        return { action: "error" as const, text: "Failed" };
      default:
        return { action: "muted" as const, text: "Ready" };
    }
  };

  const badgeProps = getBadgeProps();
  const currentStepNumber =
    currentProgress?.task === task.id ? currentProgress.current_step : 1;

  return (
    <SafeAreaView className="flex-1 flex-row gap-4 bg-gray-50 px-5 py-5">
      {/* Camera Feed */}
      <Card variant="outline" className="w-8/12">
        <View className="mb-4 flex-row items-center gap-2">
          <RadioIcon size={20} />
          <Heading size="xl">Live Camera</Heading>
        </View>

        <View className="flex-1">
          <CameraStream />
        </View>
      </Card>

      <View className="flex-1 gap-4">
        <Card variant="outline">
          <View className="mb-4 flex-row items-center justify-between">
            <View className="flex-row items-center gap-2">
              <ActivityIcon size={20} />
              <Heading size="xl">Task</Heading>
            </View>
            <Badge size="lg" variant="solid" action={badgeProps.action}>
              <BadgeText>{badgeProps.text}</BadgeText>
            </Badge>
          </View>

          <View className="mb-3 flex-row items-center justify-between">
            <Text className="font-heading text-lg">{task.title}</Text>
            <Text className="font-heading text-sm">{task.progress}%</Text>
          </View>

          <Progress
            value={task.progress}
            orientation="horizontal"
            className="mb-2"
          >
            <ProgressFilledTrack />
          </Progress>
          <Text className="mb-6 font-heading">{`Current: ${task.currentStep}`}</Text>

          {/* Steps Section - Always Visible */}
          <View className="mb-4">
            <Text className="mb-3 text-lg">
              All Steps ({currentStepNumber}/{task.totalSteps})
            </Text>
            <FlatList
              data={task.steps}
              keyExtractor={(item) => item.title}
              renderItem={({ item }) => (
                <TaskStep title={item.title} status={item.status} />
              )}
              ItemSeparatorComponent={() => <View className="h-1" />}
            />
          </View>
        </Card>

        <Card variant="outline">
          <View className="mb-4 flex-row items-center gap-2">
            <SlidersHorizontalIcon size={20} />
            <Heading size="xl">Controls</Heading>
          </View>

          <View className="mb-6 h-20 flex-row gap-4">
            {task.status === "not-started" || task.status === "failed" ? (
              <Button
                className="h-full flex-1 flex-col"
                action="positive"
                onPress={handleStartTask}
              >
                <ButtonIcon as={PlayIcon} />
                <ButtonText>Start</ButtonText>
              </Button>
            ) : (
              <Button className="h-full flex-1 flex-col" disabled>
                <ButtonIcon as={PauseIcon} />
                <ButtonText>Pause</ButtonText>
              </Button>
            )}
            <Button
              action="negative"
              className="h-full flex-1 flex-col"
              disabled
            >
              <ButtonIcon as={SquareIcon} />
              <ButtonText>Stop</ButtonText>
            </Button>
            <Button
              action="secondary"
              className="h-full flex-1 flex-col"
              disabled
            >
              <ButtonIcon as={MicIcon} />
              <ButtonText>Voice</ButtonText>
            </Button>
          </View>

          <Heading className="mb-3">Primitive Actions</Heading>
          <View className="flex-row gap-4">
            <Button variant="outline" className="h-20">
              <ButtonText>Grasp</ButtonText>
            </Button>
            <Button variant="outline" className="h-20">
              <ButtonText>Place</ButtonText>
            </Button>
          </View>
        </Card>
      </View>
    </SafeAreaView>
  );
}
