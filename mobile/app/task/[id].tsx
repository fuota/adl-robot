import React, { useEffect, useState, useRef, useCallback } from "react";
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
import { useTaskWebSocket } from "@/hooks/useTaskWebSocket";

// Task definitions
const taskDefinitions: Record<string, { title: string; steps: readonly { title: string; status: string }[] }> = {
  "1": {
    title: "Set up the table",
    steps: [
      { title: "Arrange plate on table", status: "not-started" },
      { title: "Arrange spoon on table", status: "not-started" },
      { title: "Arrange milk on the table", status: "not-started" },
      { title: "Arrange glass on the table", status: "not-started" },
    ],
  },
  "2": {
    title: "Prepare medicine",
    steps: [
      { title: "Pour water and place water bottle", status: "not-started" },
      { title: "Place cup", status: "not-started" },
      { title: "Place plate", status: "not-started" },
      { title: "Place and pour medicine bottle", status: "not-started" },
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

  // Connect to WebSocket for real-time task updates
  useTaskWebSocket(taskId);

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

  // Speech support for voice navigation on task pages
  let NativeVoice: any = null;
  try {
    // eslint-disable-next-line @typescript-eslint/no-var-requires
    const voiceModule = require("@react-native-voice/voice");
    NativeVoice = voiceModule.default || voiceModule;
  } catch (e) {
    NativeVoice = null;
  }

  const recognitionRef = useRef<any | null>(null);
  const nativeListeningRef = useRef(false);
  const [listening, setListening] = useState(false);
  const [transcript, setTranscript] = useState("");
  const latestTranscriptRef = useRef<string>("");
  const recognitionTimeoutRef = useRef<any>(null);

  // Helper function to stop speech recognition
  const stopSpeechRecognition = async () => {
    if (NativeVoice && Platform.OS !== "web" && nativeListeningRef.current) {
      try {
        if (typeof NativeVoice.stop === 'function') {
          await NativeVoice.stop();
        }
      } catch (e) {
        console.error("Error stopping voice:", e);
      }
      nativeListeningRef.current = false;
      setListening(false);
      if (recognitionTimeoutRef.current) {
        clearTimeout(recognitionTimeoutRef.current);
        recognitionTimeoutRef.current = null;
      }
    }
  };

  // Handle voice command mapping - use useCallback to avoid closure issues
  const handleVoiceMapping = useCallback(async (text: string) => {
    const t = String(text).toLowerCase().trim();
    setTranscript(text);
    
    // Check for back/home commands - support various phrases
    if (
      t === "back" || 
      t === "go back" || 
      t.includes("go back") || 
      t.includes("go to home") || 
      t.includes("go home") ||
      t.includes("home page") ||
      t === "home" ||
      t === "return" ||
      t.includes("return home") ||
      t.includes("return to home")
    ) {
      router.push('/');
      return;
    }
    // You can add more voice commands here for task-specific actions
  }, [router]);

  // Set up Voice event listeners
  useEffect(() => {
    if (NativeVoice && Platform.OS !== "web") {
      // Handle partial results (interim results while speaking)
      const onSpeechPartialResults = (event: any) => {
        const text = (event?.value && event.value[0]) || "";
        if (text) {
          latestTranscriptRef.current = text;
          setTranscript(text);
          console.log("Partial result:", text);
        }
      };

      // Handle final results (complete phrase)
      const onSpeechResults = async (event: any) => {
        const text = (event?.value && event.value[0]) || latestTranscriptRef.current || "";
        console.log("Final result:", text);
        // Stop recognition first
        await stopSpeechRecognition();
        // Clear the partial results
        latestTranscriptRef.current = "";
        // Check if we got any text
        if (!text || text.trim().length === 0) {
          setTranscript("");
          return;
        }
        // Then process the transcript
        handleVoiceMapping(text);
      };
      
      const onSpeechError = async (event: any) => {
        await stopSpeechRecognition();
        latestTranscriptRef.current = "";
        setTranscript("");
        
        const errorCode = event?.error?.code || event?.code;
        const errorMessage = event?.error?.message || event?.message || "Unknown error";
        
        // Handle specific error cases
        if (errorCode === 6 || errorMessage.includes("no speech") || errorMessage.includes("no match")) {
          // No speech detected - silently fail or show a brief message
          console.log("No speech detected");
        } else if (errorCode === 7 || errorMessage.includes("recognition not available")) {
          console.error("Speech recognition not available");
        } else {
          console.error("Speech error:", errorMessage);
        }
      };
      
      // Handle when speech ends - check if we got any results
      const onSpeechEnd = async () => {
        console.log("Speech ended, waiting for final results...");
        // Set a timeout to handle case where no results come after speech ends
        recognitionTimeoutRef.current = setTimeout(async () => {
          // If we still have no transcript after a delay, it means no speech was detected
          if (!latestTranscriptRef.current && nativeListeningRef.current) {
            await stopSpeechRecognition();
            setTranscript("");
          }
        }, 2000); // Wait 2 seconds for results
      };

      NativeVoice.onSpeechPartialResults = onSpeechPartialResults;
      NativeVoice.onSpeechResults = onSpeechResults;
      NativeVoice.onSpeechError = onSpeechError;
      NativeVoice.onSpeechEnd = onSpeechEnd;

      return () => {
        // Cleanup
        stopSpeechRecognition();
        if (recognitionTimeoutRef.current) {
          clearTimeout(recognitionTimeoutRef.current);
          recognitionTimeoutRef.current = null;
        }
        if (NativeVoice && typeof NativeVoice.destroy === 'function') {
          NativeVoice.destroy().catch(() => {});
        }
        if (NativeVoice && typeof NativeVoice.removeAllListeners === 'function') {
          NativeVoice.removeAllListeners();
        }
      };
    }
  }, [NativeVoice, handleVoiceMapping]);

  const startListening = async () => {
    const globalAny: any = typeof window !== "undefined" ? window : null;

    // 1. Try native voice recognition first (iOS/Android native builds)
    if (NativeVoice && Platform.OS !== "web") {
      if (!listening) {
        try {
          // Check if NativeVoice has the start method
          if (!NativeVoice || typeof NativeVoice.start !== 'function') {
            console.error("Voice module:", NativeVoice);
            return;
          }

          // Stop any existing recognition first to avoid "already started" error
          if (nativeListeningRef.current && NativeVoice && typeof NativeVoice.stop === 'function') {
            try {
              await NativeVoice.stop();
            } catch (stopError) {
              // Ignore stop errors, just continue
              console.log("Stop error (ignored):", stopError);
            }
            nativeListeningRef.current = false;
          }

          // Small delay to ensure previous recognition is fully stopped
          await new Promise(resolve => setTimeout(resolve, 100));

          console.log("Starting voice recognition...");
          await NativeVoice.start("en-US");
          setListening(true);
          nativeListeningRef.current = true;
          setTranscript("");
          latestTranscriptRef.current = "";
          console.log("Voice recognition started successfully");
        } catch (e: any) {
          console.error("Voice start error:", e);
          setListening(false);
          nativeListeningRef.current = false;
          setTranscript(`Error: ${e?.message || String(e)}`);
        }
      } else {
        // Stop native recognition
        await stopSpeechRecognition();
      }
      return;
    }

    // 2. Try web SpeechRecognition API (works in web browsers)
    const SpeechRecognition = globalAny?.SpeechRecognition || globalAny?.webkitSpeechRecognition;
    if (SpeechRecognition) {
      if (!listening) {
        try {
          const recognition = new SpeechRecognition();
          recognition.lang = "en-US";
          recognition.interimResults = false;
          recognition.maxAlternatives = 1;
          recognition.continuous = false;

          recognition.onresult = async (ev: any) => {
            const transcript = String(ev.results[0][0].transcript);
            await handleVoiceMapping(transcript);
          };

          recognition.onerror = (e: any) => {
            console.error("Speech recognition error", e);
            setListening(false);
          };

          recognition.onend = () => {
            setListening(false);
            recognitionRef.current = null;
          };

          recognitionRef.current = recognition;
          recognition.start();
          setListening(true);
        } catch (e: any) {
          console.error("Failed to start speech recognition:", e);
          setListening(false);
          recognitionRef.current = null;
        }
      } else {
        // Stop web recognition
        try {
          recognitionRef.current?.stop?.();
        } catch (e) {
          // ignore
        }
        recognitionRef.current = null;
        setListening(false);
      }
      return;
    }

    // 3. Fallback: Show helpful message
    if (Platform.OS === "web") {
      console.log("Speech recognition not available in this browser");
    } else {
      console.log("Native speech recognition requires a custom development build.");
    }
  };

  const stopListening = async () => {
    await stopSpeechRecognition();
    if (recognitionRef.current) {
      try { recognitionRef.current.stop(); } catch (e) { }
      recognitionRef.current = null;
    }
    setListening(false);
    setTranscript("");
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
            <Button 
              action="secondary" 
              className="h-full flex-1 flex-col" 
              onPress={async () => {
                console.log("Voice button pressed, listening:", listening);
                if (listening) {
                  await stopListening();
                } else {
                  await startListening();
                }
              }}
            >
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

