import React, { useEffect, useState } from "react";
import CameraStream from "@/components/CameraStream";
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
import { FlatList, View, Pressable, DeviceEventEmitter } from "react-native";
import { SafeAreaView } from "react-native-safe-area-context";
import { useRouter } from "expo-router";

const initialSteps = [
  { title: "Pour water and place water bottle", status: "not-started" },
  { title: "Place cup", status: "not-started" },
  { title: "Place plate", status: "not-started" },
  { title: "Place and pour medicine bottle", status: "not-started" },
] as const;

export default function Task1() {
  const router = useRouter();
  const taskTitle = "Prepare medicine";
  const taskId = "1";

  type StepStatus = "completed" | "in-progress" | "not-started";
  const [steps, setSteps] = useState(() => 
    initialSteps.map((s) => ({ title: s.title, status: s.status })) as { title: string; status: StepStatus }[]
  );

  useEffect(() => {
    setSteps((prev) => prev.map((s, i) => ({ ...s, status: i === 0 ? "in-progress" : s.status })));
  }, []);

  const completedCount = steps.filter((s) => s.status === "completed").length;
  const inProgressCount = steps.filter((s) => s.status === "in-progress").length;
  const total = steps.length;
  const currentIndex = steps.findIndex((s) => s.status === "in-progress");
  const progress = Math.round((completedCount / total) * 100);

  const markStepComplete = (index: number) => {
    setSteps((prev) => {
      const next = prev.map((s) => ({ ...s }));
      if (next[index]) next[index].status = "completed";
      if (next[index + 1]) next[index + 1].status = "in-progress";
      return next;
    });
  };

  const advanceStep = () => {
    const idx = steps.findIndex((s) => s.status === "in-progress");
    if (idx === -1) {
      setSteps((prev) => prev.map((s, i) => ({ ...s, status: i === 0 ? "in-progress" : s.status })));
      return;
    }
    markStepComplete(idx);
  };

  useEffect(() => {
    const handler = (payload: any) => {
      const p = payload?.detail ?? payload;
      if (!p) return;
      if (p.taskId && String(p.taskId) !== taskId) return;
      if (p.action === "advance") {
        advanceStep();
        return;
      }
      if (typeof p.stepIndex === "number") {
        setSteps((prev) => {
          const next = prev.map((s) => ({ ...s }));
          if (next[p.stepIndex]) next[p.stepIndex].status = p.status || next[p.stepIndex].status;
          return next;
        });
      }
    };

    let sub: any = null;
    try {
      sub = DeviceEventEmitter.addListener("taskStepUpdate", handler);
    } catch (e) {
      sub = null;
    }

    const webHandler = (e: any) => handler(e.detail ?? e);
    if (typeof window !== "undefined" && window.addEventListener) {
      window.addEventListener("taskStepUpdate", webHandler);
    }

    return () => {
      sub && sub.remove && sub.remove();
      if (typeof window !== "undefined" && window.removeEventListener) {
        window.removeEventListener("taskStepUpdate", webHandler);
      }
    };
  }, []);

  return (
    <SafeAreaView className="flex-1 flex-row gap-4 bg-gray-50 px-5 py-5">
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
              <Pressable onPress={() => router.push('/')} className="px-2 py-1 rounded">
                <Text className="text-blue-600">← Back</Text>
              </Pressable>
              <ActivityIcon size={20} />
              <Heading size="xl">Task</Heading>
            </View>
            <Badge size="lg" variant="solid" action="success">
              <BadgeText>Running</BadgeText>
            </Badge>
          </View>

          <View className="mb-3 flex-row items-center justify-between">
            <Text className="font-heading text-lg">{taskTitle}</Text>
            <Text className="font-heading text-sm">{progress}%</Text>
          </View>

          <Progress value={progress} orientation="horizontal" className="mb-2">
            <ProgressFilledTrack />
          </Progress>
          <Text className="mb-6 font-heading">Step {Math.min(currentIndex + 1, total)} of {total}: {steps[currentIndex]?.title ?? steps[0].title}</Text>

          <Accordion isCollapsible>
            <AccordionItem value="all-steps">
              <AccordionHeader>
                <AccordionTrigger className="p-0">
                  {({ isExpanded }: { isExpanded: boolean }) => {
                    return (
                      <>
                        <AccordionTitleText className="text-lg">All Steps ({completedCount}/{total})</AccordionTitleText>

                        {isExpanded ? (
                          <AccordionIcon as={ChevronUpIcon} className="ml-3" />
                        ) : (
                          <AccordionIcon as={ChevronDownIcon} className="ml-3" />
                        )}
                      </>
                    );
                  }}
                </AccordionTrigger>
              </AccordionHeader>
              <AccordionContent className="p-0">
                <FlatList
                  data={steps}
                  keyExtractor={(item) => item.title}
                  renderItem={({ item, index }) => (
                    <Pressable onPress={() => markStepComplete(index)}>
                      <TaskStep {...item} />
                    </Pressable>
                  )}
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

          <View className="mb-6 h-20 flex-row gap-4 items-stretch">
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
        </Card>
      </View>
    </SafeAreaView>
  );
}
