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
import { FlatList, View, Platform } from "react-native";
import { SafeAreaView } from "react-native-safe-area-context";
import { useEffect, useState, useRef, useCallback } from "react";

export default function TaskScreen() {
  // Read selected id from TaskContext
  const { selectedId, tasks, startTask, stopTask, currentProgress } = useTask();
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
  
  const handleStopTask = () => {
    stopTask(task.id);
  };

  // Speech support for voice commands on task pages
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
    
    // Check for start command - support various phrases
    if (
      t === "start" ||
      t === "start task" ||
      t.includes("start the task") ||
      t.includes("begin") ||
      t.includes("begin task") ||
      t === "go" ||
      t === "execute" ||
      t.includes("execute task")
    ) {
      // Only start if task is not already in progress
      if (!task || task.status === "not-started" || task.status === "failed" || task.status === "completed") {
        console.log("[Voice] Starting task via voice command");
        startTask(task.id);
      } else {
        console.log("[Voice] Task is already in progress, cannot start");
      }
      return;
    }
  }, [task, startTask]);

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
        const errorCode = event?.error?.code;
        const errorMessage = event?.error?.message || "";
        if (errorCode === 6 || errorMessage.includes("no speech") || errorMessage.includes("no match")) {
          // No speech detected - silently fail or show a brief message
          console.log("No speech detected");
        } else if (errorCode === 9) {
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
          if (!latestTranscriptRef.current) {
            await stopSpeechRecognition();
          }
        }, 1000);
      };

      NativeVoice.onSpeechPartialResults = onSpeechPartialResults;
      NativeVoice.onSpeechResults = onSpeechResults;
      NativeVoice.onSpeechError = onSpeechError;
      NativeVoice.onSpeechEnd = onSpeechEnd;

      return () => {
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
          nativeListeningRef.current = true;
          setListening(true);
          console.log("Voice recognition started successfully");
        } catch (e: any) {
          console.error("Voice start error:", e);
          setListening(false);
          nativeListeningRef.current = false;
        }
      } else {
        await stopSpeechRecognition();
      }
      return;
    }

    // 2. Try web SpeechRecognition API (works in web browsers)
    const SpeechRecognition = globalAny?.SpeechRecognition || globalAny?.webkitSpeechRecognition;
    if (SpeechRecognition) {
      try {
        if (recognitionRef.current) {
          recognitionRef.current.stop();
          recognitionRef.current = null;
        }

        const recognition = new SpeechRecognition();
        recognition.continuous = false;
        recognition.interimResults = true;
        recognition.lang = "en-US";

        recognition.onresult = async (event: any) => {
          const transcript = Array.from(event.results)
            .map((result: any) => result[0].transcript)
            .join("");
          if (event.results[event.results.length - 1].isFinal) {
            await handleVoiceMapping(transcript);
          }
        };

        recognition.onerror = (event: any) => {
          console.error("Speech recognition error", event);
        };

        recognition.onend = () => {
          recognitionRef.current = null;
          setListening(false);
        };

        recognition.start();
        recognitionRef.current = recognition;
        setListening(true);
      } catch (e) {
        console.error("Failed to start speech recognition:", e);
        setListening(false);
      }
      return;
    }

    // 3. Fallback: No speech recognition available
    if (Platform.OS === "web") {
      console.log("Speech recognition not available in this browser");
    } else {
      console.log("Native speech recognition requires a custom development build.");
    }
    setListening(false);
    await stopSpeechRecognition();
  };

  const stopListening = async () => {
    await stopSpeechRecognition();
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

        <View>
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
            {/* Start/Stop button - changes based on task status */}
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
              <Button
                className="h-full flex-1 flex-col"
                action="negative"
                onPress={handleStopTask}
              >
                <ButtonIcon as={SquareIcon} />
                <ButtonText>Stop</ButtonText>
              </Button>
            )}
            {/* Voice button - always visible */}
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
