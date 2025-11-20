import React, { useEffect, useState, useRef } from "react";
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
import { FlatList, View, Pressable, DeviceEventEmitter, Platform } from "react-native";
import { SafeAreaView } from "react-native-safe-area-context";
import { useRouter, useLocalSearchParams } from "expo-router";

// Task definitions
const taskDefinitions: Record<string, { title: string; steps: readonly { title: string; status: string }[] }> = {
  "1": {
    title: "Prepare medicine",
    steps: [
      { title: "Pour water and place water bottle", status: "not-started" },
      { title: "Place cup", status: "not-started" },
      { title: "Place plate", status: "not-started" },
      { title: "Place and pour medicine bottle", status: "not-started" },
    ],
  },
  "2": {
    title: "Set up the table",
    steps: [
      { title: "Arrange plate on table", status: "not-started" },
      { title: "Arrange spoon on table", status: "not-started" },
      { title: "Arrange milk on the table", status: "not-started" },
      { title: "Arrange glass on the table", status: "not-started" },
    ],
  },
  "3": {
    title: "Organize books",
    steps: [
      { title: "Pick up the book", status: "not-started" },
      { title: "Place book appropriately", status: "not-started" },
      { title: "Pick up disarray objects", status: "not-started" },
      { title: "Arrange the objects neatly", status: "not-started" },
    ],
  },
};

export default function TaskScreen() {
  const router = useRouter();
  const { id } = useLocalSearchParams<{ id: string }>();
  const taskId = id || "1";
  
  const taskDef = taskDefinitions[taskId] || taskDefinitions["1"];
  const taskTitle = taskDef.title;
  const initialSteps = taskDef.steps;

  type StepStatus = "completed" | "in-progress" | "not-started";
  const [steps, setSteps] = useState(() => 
    initialSteps.map((s) => ({ title: s.title, status: s.status })) as { title: string; status: StepStatus }[]
  );

  useEffect(() => {
    // Initialize first step as in-progress
    setSteps((prev) => prev.map((s, i) => ({ ...s, status: i === 0 ? "in-progress" : s.status })));
  }, [taskId]);

  const completedCount = steps.filter((s) => s.status === "completed").length;
  const inProgressCount = steps.filter((s) => s.status === "in-progress").length;
  const total = steps.length;
  const currentIndex = steps.findIndex((s) => s.status === "in-progress");
  const progress = Math.round((completedCount / total) * 100);

  const advanceStep = () => {
    const idx = steps.findIndex((s) => s.status === "in-progress");
    if (idx === -1) {
      setSteps((prev) => prev.map((s, i) => ({ ...s, status: i === 0 ? "in-progress" : s.status })));
      return;
    }
    // Mark current step as completed and next as in-progress
    setSteps((prev) => {
      const next = prev.map((s) => ({ ...s }));
      if (next[idx]) next[idx].status = "completed";
      if (next[idx + 1]) next[idx + 1].status = "in-progress";
      return next;
    });
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
  }, [taskId]);

  // Speech support for task 3 (optional - can be added to all tasks if needed)
  let NativeVoice: any = null;
  try {
    // eslint-disable-next-line @typescript-eslint/no-var-requires
    NativeVoice = require("@react-native-voice/voice");
  } catch (e) {
    NativeVoice = null;
  }

  const recognitionRef = useRef<any | null>(null);
  const nativeListeningRef = useRef(false);
  const [listening, setListening] = useState(false);
  const [transcript, setTranscript] = useState("");

  const handleVoiceMapping = (text: string) => {
    const t = text.toLowerCase();
    if (t.includes("go back") || t.includes("go to home") || t.includes("home") || t === "back") {
      router.push('/');
    }
  };

  const startListening = () => {
    if (NativeVoice && Platform.OS !== "web") {
      try {
        NativeVoice.onSpeechResults = (event: any) => {
          const text = (event?.value && event.value[0]) || "";
          setTranscript(text);
          handleVoiceMapping(text);
          setListening(false);
          nativeListeningRef.current = false;
        };
        NativeVoice.onSpeechError = (event: any) => {
          console.error("Native voice error", event);
          setListening(false);
          nativeListeningRef.current = false;
        };
        NativeVoice.start("en-US");
        setListening(true);
        nativeListeningRef.current = true;
      } catch (e) {
        console.error(e);
        alert("Failed to start native speech. Rebuild the dev client with the native module.");
      }
      return;
    }

    if (typeof window === "undefined") {
      alert("Speech is only supported in web or native builds.");
      return;
    }

    const SpeechRecognition = (window as any).SpeechRecognition || (window as any).webkitSpeechRecognition;
    if (!SpeechRecognition) {
      alert("Speech recognition not supported in this browser.");
      return;
    }

    try {
      const recognition = new SpeechRecognition();
      recognition.lang = "en-US";
      recognition.interimResults = false;
      recognition.maxAlternatives = 1;

      recognition.onresult = (event: any) => {
        const text = event.results[0][0].transcript;
        setTranscript(text);
        handleVoiceMapping(text);
      };

      recognition.onerror = (event: any) => {
        console.error("Speech recognition error", event);
        setListening(false);
      };

      recognition.onend = () => {
        setListening(false);
      };

      recognitionRef.current = recognition;
      setListening(true);
      recognition.start();
    } catch (e) {
      console.error(e);
      alert("Failed to start speech recognition.");
    }
  };

  const stopListening = () => {
    if (recognitionRef.current) {
      try { recognitionRef.current.stop(); } catch (e) { }
      recognitionRef.current = null;
    }
    if (NativeVoice && nativeListeningRef.current) {
      try { NativeVoice.stop(); } catch (e) { }
      nativeListeningRef.current = false;
    }
    setListening(false);
  };

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

          {/* Steps Section - Always Visible */}
          <View className="mb-4">
            <Text className="text-lg mb-3">All Steps ({completedCount}/{total})</Text>
            <FlatList
              data={steps}
              keyExtractor={(item) => item.title}
              renderItem={({ item }) => (
                <TaskStep {...item} />
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

          <View className="mb-6 h-20 flex-row gap-4 items-stretch">
            <Button className="h-full flex-1 flex-col">
              <ButtonIcon as={PauseIcon} />
              <ButtonText>Pause</ButtonText>
            </Button>
            <Button action="negative" className="h-full flex-1 flex-col">
              <ButtonIcon as={SquareIcon} />
              <ButtonText>Stop</ButtonText>
            </Button>
            <Button action="secondary" className="h-full flex-1 flex-col" onPress={() => (listening ? stopListening() : startListening())}>
              <ButtonIcon as={(_props: any) => <MicIcon {..._props} size={28} />} />
              <ButtonText>{listening ? "Listening..." : "Voice"}</ButtonText>
            </Button>
          </View>
          {transcript ? <Text className="mt-2 text-sm">Heard: {transcript}</Text> : null}
        </Card>
      </View>
    </SafeAreaView>
  );
}

