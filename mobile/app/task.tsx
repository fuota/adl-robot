import TaskStep from "@/components/tasks/TaskStep";
import {
  Accordion,
  AccordionContent,
  AccordionHeader,
  AccordionIcon,
  AccordionItem,
  AccordionTitleText,
  AccordionTrigger,
} from "@/components/ui/accordion";
import { Badge, BadgeText } from "@/components/ui/badge";
import { Button, ButtonIcon, ButtonText } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Heading } from "@/components/ui/heading";
import { Progress, ProgressFilledTrack } from "@/components/ui/progress";
import { Text } from "@/components/ui/text";
import {
  ActivityIcon,
  ChevronDownIcon,
  ChevronUpIcon,
  MicIcon,
  PauseIcon,
  RadioIcon,
  SlidersHorizontalIcon,
  SquareIcon,
} from "lucide-react-native";
import { FlatList, View } from "react-native";
import { SafeAreaView } from "react-native-safe-area-context";
import { WebView } from "react-native-webview";

const mockSteps = [
  { title: "Step 1", status: "completed" },
  { title: "Step 2", status: "in-progress" },
  { title: "Step 3", status: "not-started" },
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

        <WebView
          source={{ uri: "http://10.226.122.218:5000/camera/viewer" }}
          style={{
            flex: 1,
            backgroundColor: "#000",
            borderRadius: 8,
          }}
          allowsInlineMediaPlayback={true}
          mediaPlaybackRequiresUserAction={false}
          javaScriptEnabled={true}
          domStorageEnabled={true}
          startInLoadingState={true}
          scalesPageToFit={false}
          bounces={false}
          scrollEnabled={false}
          showsHorizontalScrollIndicator={false}
          showsVerticalScrollIndicator={false}
          onError={(syntheticEvent) => {
            const { nativeEvent } = syntheticEvent;
            console.warn("WebView error: ", nativeEvent);
          }}
          onHttpError={(syntheticEvent) => {
            const { nativeEvent } = syntheticEvent;
            console.warn("WebView HTTP error: ", nativeEvent);
          }}
        />
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
              Put the cup on the shelf
            </Text>
            <Text className="font-heading text-sm">40%</Text>
          </View>

          <Progress value={40} orientation="horizontal" className="mb-2">
            <ProgressFilledTrack />
          </Progress>
          <Text className="mb-6 font-heading">Step 2 of 3: Grasping book</Text>

          <Accordion isCollapsible>
            <AccordionItem value="all-steps">
              <AccordionHeader>
                <AccordionTrigger className="p-0">
                  {({ isExpanded }: { isExpanded: boolean }) => {
                    return (
                      <>
                        <AccordionTitleText className="text-lg">
                          All Steps (2/3)
                        </AccordionTitleText>

                        {isExpanded ? (
                          <AccordionIcon as={ChevronUpIcon} className="ml-3" />
                        ) : (
                          <AccordionIcon
                            as={ChevronDownIcon}
                            className="ml-3"
                          />
                        )}
                      </>
                    );
                  }}
                </AccordionTrigger>
              </AccordionHeader>
              <AccordionContent className="p-0">
                <FlatList
                  data={mockSteps}
                  keyExtractor={(item) => item.title}
                  renderItem={({ item }) => <TaskStep {...item} />}
                  ItemSeparatorComponent={() => <View className="h-1" />}
                />
              </AccordionContent>
            </AccordionItem>
          </Accordion>
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
