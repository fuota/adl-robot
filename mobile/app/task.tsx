import CameraStream from "@/components/CameraStream";
import TaskStep from "@/components/tasks/TaskStep";
import { Badge, BadgeText } from "@/components/ui/badge";
import { Button, ButtonIcon, ButtonText } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Heading } from "@/components/ui/heading";
import { Progress, ProgressFilledTrack } from "@/components/ui/progress";
import { Text } from "@/components/ui/text";
import {
  ActivityIcon,
  MicIcon,
  PauseIcon,
  RadioIcon,
  SlidersHorizontalIcon,
  SquareIcon,
} from "lucide-react-native";
import { FlatList, View } from "react-native";
import { useTask } from "@/contexts/TaskContext";
import { SafeAreaView } from "react-native-safe-area-context";

const tasks = [
  {
    id: "1",
    title: "Prepare medicine",
    description: "Prepare the medicine for the patient.",
    progress: 100,
    currentStep: "Pour water and place water bottle",
    steps: [
      { title: "Pour water and place water bottle", status: "in-progress" },
      { title: "Place cup", status: "not-started" },
      { title: "Place plate", status: "not-started" },
      { title: "Place and pour medicine bottle", status: "not-started" },
    ],
  },
  {
    id: "2",
    title: "Set up the table",
    description: "Set the table for the meal.",
    progress: 0,
    currentStep: "Place plates and cutlery",
    steps: [
      { title: "Place plates and cutlery", status: "not-started" },
      { title: "Place glasses", status: "not-started" },
    ],
  },
  {
    id: "3",
    title: "Organize books",
    description: "Organize the books on the shelf.",
    progress: 0,
    currentStep: "Gather books",
    steps: [
      { title: "Gather books", status: "not-started" },
      { title: "Arrange on shelf", status: "not-started" },
    ],
  },
] as const;

export default function TaskScreen() {
  // Read selected id from TaskContext (set by Home) if available. Fallback to URL query for web.
  let id = "1";
  try {
    const { selectedId } = useTask();
    if (selectedId) id = selectedId;
    else if (typeof window !== "undefined") id = new URLSearchParams(window.location.search).get("id") ?? "1";
  } catch (e) {
    // no context available, fallback to URL
    if (typeof window !== "undefined") id = new URLSearchParams(window.location.search).get("id") ?? "1";
  }

  const task = tasks.find((t) => t.id === id) ?? tasks[0];

  const mockSteps: Array<{ title: string; status: string }> = task.steps as any;

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
            <Badge size="lg" variant="solid" action="success">
              <BadgeText>Running</BadgeText>
            </Badge>
          </View>

          <View className="mb-3 flex-row items-center justify-between">
            <Text className="font-heading text-lg">{task.title}</Text>
            <Text className="font-heading text-sm">{task.progress}%</Text>
          </View>

          <Progress value={task.progress} orientation="horizontal" className="mb-2">
            <ProgressFilledTrack />
          </Progress>
          <Text className="mb-6 font-heading">{`Current: ${task.currentStep}`}</Text>

          {/* Steps Section - Always Visible */}
          <View className="mb-4">
            <Text className="text-lg mb-3">All Steps (1/4)</Text>
            <FlatList
              data={mockSteps}
              keyExtractor={(item) => item.title}
              renderItem={({ item }) => <TaskStep {...(item as any)} />}
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
            <Button className="h-full flex-1 flex-col">
              <ButtonIcon as={PauseIcon} />
              <ButtonText>Pause</ButtonText>
            </Button>
            <Button action="negative" className="h-full flex-1 flex-col">
              <ButtonIcon as={SquareIcon} />
              <ButtonText>Stop</ButtonText>
            </Button>
            <Button action="secondary" className="h-full flex-1 flex-col">
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