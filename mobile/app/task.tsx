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
import { SafeAreaView } from "react-native-safe-area-context";

const mockSteps = [
  { title: "Pour water and place water bottle", status: "in-progress" },
  { title: "Place cup", status: "not-started" },
  { title: "Place plate", status: "not-started" },
  { title: "Place and pour medicine bottle", status: "not-started" },
] as const;

export default function TaskScreen() {
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
            <Text className="font-heading text-lg">
              Prepare medicine
            </Text>
            <Text className="font-heading text-sm">100%</Text>
          </View>

          <Progress value={100} orientation="horizontal" className="mb-2">
            <ProgressFilledTrack />
          </Progress>
          <Text className="mb-6 font-heading">Step 1 of 4: Pour water and place water bottle</Text>

          {/* Steps Section - Always Visible */}
          <View className="mb-4">
            <Text className="text-lg mb-3">All Steps (1/4)</Text>
            <FlatList
              data={mockSteps}
              keyExtractor={(item) => item.title}
              renderItem={({ item }) => <TaskStep {...item} />}
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