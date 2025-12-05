import React, { useEffect, useState, useRef, useCallback } from "react";
import CameraStream from "@/components/CameraStream";
import TaskStep from "@/components/tasks/TaskStep";
import { Badge, BadgeText } from "@/components/ui/badge";
import { Button, ButtonIcon, ButtonText } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Heading } from "@/components/ui/heading";
import { Progress, ProgressFilledTrack } from "@/components/ui/progress";
import { Switch } from "@/components/ui/switch";
import { Text } from "@/components/ui/text";
import {
  Activity as ActivityIcon,
  Hand as HandIcon,
  Mic as MicIcon,
  Play as PlayIcon,
  Radio as RadioIcon,
  SlidersHorizontal as SlidersHorizontalIcon,
  Square as SquareIcon,
} from "lucide-react-native";
import { FlatList, View, Pressable, Platform } from "react-native";
import { SafeAreaView } from "react-native-safe-area-context";
import { useRouter, useLocalSearchParams } from "expo-router";
import { useTask } from "@/contexts/TaskContext";

// Task definitions
const taskDefinitions: Record<string, { title: string; steps: readonly { title: string; status: string }[] }> = {
  "1": {
    title: "Set up the table",
    steps: [
      { title: "Arrange plate on table", status: "not-started" },
      { title: "Arrange fork on table", status: "not-started" },
      { title: "Arrange spoon on the table", status: "not-started" },
      //{ title: "Arrange glass on the table", status: "not-started" },
    ],
  },
  "2": {
    title: "Prepare medicine",
    steps: [
      { title: "Place water cup in serving area", status: "not-started" },
      { title: "Place plate in serving area", status: "not-started" },
      { title: "Pour water into cup", status: "not-started" },
      { title: "Pour medicine into plate", status: "not-started" },
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
  
  // Get task context for starting/stopping tasks via ROS
  const { startTask, stopTask, tasks, callPrimitiveAction, detectedObjects } = useTask();
  const taskFromContext = tasks.find((t) => t.id === taskId);
  
  // State for selected object
  const [selectedObjectId, setSelectedObjectId] = useState<string | null>(null);
  
  // State for showing object labels
  const [showObjects, setShowObjects] = useState(false);
  
  // Debug: Log task status for button state
  useEffect(() => {
    if (taskFromContext) {
      console.log(`[Task] Button state check: taskId=${taskId}, status=${taskFromContext.status}, progress=${taskFromContext.progress}%`);
    } else {
      console.log(`[Task] Button state check: taskId=${taskId}, taskFromContext is null/undefined`);
    }
    // Debug: Check if primitive action buttons should be visible
    console.log(`[Task] Primitive actions visibility: taskId=${taskId}, shouldShow=${taskId === "1" || taskId === "2" || taskId === "3"}`);
  }, [taskId, taskFromContext?.status, taskFromContext?.progress]);
  
  const taskDef = taskDefinitions[taskId] || taskDefinitions["1"];
  const taskTitle = taskDef.title;
  const initialSteps = taskDef.steps;

  // Derive a dynamic badge state from the task status
  const getBadgeProps = () => {
    const status = taskFromContext?.status || "not-started";
    switch (status) {
      case "in_progress":
        return { action: "warning" as const, text: "Running" };
      case "completed":
        return { action: "success" as const, text: "Finished" };
      case "failed":
        return { action: "error" as const, text: "Failed" };
      case "not-started":
      default:
        return { action: "muted" as const, text: "Idle" };
    }
  };
  const badgeProps = getBadgeProps();
  
  // Handler to start the task via ROS
  const handleStartTask = () => {
    console.log(`[Task] Starting task ${taskId} via ROS`);
    startTask(taskId);
  };
  
  // Handler to stop the task
  const handleStopTask = () => {
    console.log(`[Task] Stopping task ${taskId}`);
    stopTask(taskId);
  };

  // Handler for primitive actions (Grasp, Lift, Drop)
  const handlePrimitiveAction = (action: string, objectId: string) => {
    console.log(`[Task] Calling primitive action: ${action} for object ${objectId} in task ${taskId}`);
    callPrimitiveAction(action, objectId, taskId);
  };
  
  // Format object name: remove underscores, capitalize first letter of each word
  const formatObjectName = (name: string): string => {
    return name
      .split("_")
      .map((word) => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
      .join(" ");
  };

  type StepStatus = "completed" | "in_progress" | "not-started";
  
  // Use steps from TaskContext if available, otherwise use local state
  const [localSteps, setLocalSteps] = useState(() => 
    initialSteps.map((s) => ({ title: s.title, status: s.status })) as { title: string; status: StepStatus }[]
  );
  
  // Use TaskContext steps directly (updated via ROS topic subscription)
  // TaskContext subscribes to /task_progress topic directly via ROSLIB
  // This is the ONLY source of truth for task progress - no WebSocket needed
  const steps = taskFromContext?.steps || localSteps;
  const taskProgress = taskFromContext?.progress || 0;
  const totalStepsCount = taskFromContext?.totalSteps || steps.length;

  // Sync with TaskContext updates (from ROS topic subscription on native)
  useEffect(() => {
    if (taskFromContext?.steps) {
      console.log(`[Task] Syncing with TaskContext: task ${taskId}, steps:`, taskFromContext.steps.map(s => `${s.title}: ${s.status}`));
      // TaskContext has updated steps from ROS, use those
      setLocalSteps(taskFromContext.steps.map(s => ({ 
        title: s.title, 
        status: s.status as StepStatus 
      })));
    } else {
      // Initialize first step as in-progress when task changes (only if no TaskContext data)
      setLocalSteps((prev) => prev.map((s, i) => ({ ...s, status: i === 0 ? "in_progress" : "not-started" })));
    }
  }, [taskId, taskFromContext?.steps, taskFromContext?.progress]);

  const completedCount = steps.filter((s) => s.status === "completed").length;
  const currentIndex = steps.findIndex((s) => s.status === "in_progress");
  const fallbackStepNumber = currentIndex >= 0 ? currentIndex + 1 : 1;
  const activeStepNumber = taskFromContext?.currentStepNumber || fallbackStepNumber;
  const activeStepTitle =
    taskFromContext?.currentStep ||
    steps[currentIndex]?.title ||
    steps[Math.min(fallbackStepNumber - 1, steps.length - 1)]?.title ||
    "Ready";
  
  // Use TaskContext progress (updates in real-time from ROS)
  // Progress comes from ROS progress_percent field
  // Progress bar updates in real-time as ROS publishes updates
  const progress = taskProgress || 0;
  
  // Debug: Log progress updates for prepare medicine task
  useEffect(() => {
    if (taskId === "2" && taskFromContext) {
      console.log(`[Task] Prepare Medicine Progress Update:`, {
        progress: taskFromContext.progress,
        status: taskFromContext.status,
        currentStep: taskFromContext.currentStepNumber,
        totalSteps: taskFromContext.totalSteps,
      });
    }
  }, [taskId, taskFromContext?.progress, taskFromContext?.status, taskFromContext?.currentStepNumber]);

  // No WebSocket or DeviceEventEmitter needed - TaskContext handles all updates via ROS topic subscription
  // TaskContext subscribes to /task_progress topic directly and updates task state
  // The component automatically re-renders when TaskContext state changes

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
      if (!taskFromContext || taskFromContext.status === "not-started" || taskFromContext.status === "failed" || taskFromContext.status === "completed") {
        console.log("[Voice] Starting task via voice command");
        startTask(taskId);
      } else {
        console.log("[Voice] Task is already in progress, cannot start");
      }
      return;
    }
    
    // Check for stop command - support various phrases
    if (
      t === "stop" ||
      t === "stop task" ||
      t.includes("stop the task") ||
      t.includes("halt") ||
      t.includes("halt task") ||
      t === "cancel" ||
      t.includes("cancel task") ||
      t.includes("abort")
    ) {
      // Only stop if task is in progress
      if (taskFromContext && taskFromContext.status === "in_progress") {
        console.log("[Voice] Stopping task via voice command");
        stopTask(taskId);
      } else {
        console.log("[Voice] Task is not in progress, cannot stop");
      }
      return;
    }
    
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
  }, [router, taskFromContext, startTask, stopTask, taskId]);

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
        <View className="mb-4 flex-row items-center justify-between">
          <View className="flex-row items-center gap-2">
            <RadioIcon size={20} />
            <Heading size="xl">Live Camera</Heading>
          </View>
          <View className="flex-row items-center gap-2">
            <Text className="text-sm">Show Objects</Text>
            <Switch
              value={showObjects}
              onValueChange={setShowObjects}
            />
          </View>
        </View>

        <View>
          <CameraStream showObjects={showObjects} />
        </View>
        
        {/* Detected Objects - Buttons in a row */}
        <View className="mt-4">
          <Text className="text-sm font-semibold mb-2 text-gray-700">Detected Objects</Text>
          <View className="flex-row flex-wrap gap-2">
            {Object.entries(detectedObjects).map(([objectId, obj]) => (
              <Button
                key={objectId}
                size="sm"
                variant={selectedObjectId === objectId ? "solid" : "outline"}
                action={selectedObjectId === objectId ? "primary" : "secondary"}
                onPress={() => setSelectedObjectId(selectedObjectId === objectId ? null : objectId)}
              >
                <ButtonText>
                  {formatObjectName(obj.name)}
                </ButtonText>
              </Button>
            ))}
            {Object.keys(detectedObjects).length === 0 && (
              <Text className="text-sm text-gray-500">No objects detected yet</Text>
            )}
          </View>
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
            <Badge size="lg" variant="solid" action={badgeProps.action}>
              <BadgeText>{badgeProps.text}</BadgeText>
            </Badge>
          </View>

          <View className="mb-3 flex-row items-center justify-between">
            <Text className="font-heading text-lg">{taskTitle}</Text>
            <Text className="font-heading text-sm">{progress}%</Text>
          </View>

          <Progress value={progress} orientation="horizontal" className="mb-2">
            <ProgressFilledTrack />
          </Progress>
          <Text className="mb-6 font-heading">
            Step {Math.min(activeStepNumber, totalStepsCount)} of {totalStepsCount}: {activeStepTitle}
          </Text>

          {/* Steps Section - Always Visible */}
          <View className="mb-4">
            <Text className="text-lg mb-3">All Steps ({completedCount}/{totalStepsCount})</Text>
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
            {/* Start/Stop button - changes based on task status */}
            {(!taskFromContext || taskFromContext.status === "not-started" || taskFromContext.status === "failed" || taskFromContext.status === "completed") ? (
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
              action="primary" 
              className="h-full flex-1 flex-col bg-blue-600" 
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

          {/* Primitive Action Buttons - Show when object is selected */}
          {selectedObjectId && detectedObjects[selectedObjectId] && (
            <View className="mb-4">
              <Text className="text-sm font-semibold mb-3 text-gray-700">
                Primitive Actions for {formatObjectName(detectedObjects[selectedObjectId].name)}
              </Text>
              <View className="h-20 flex-row gap-4 items-stretch">
                <Button
                  className="h-full flex-1 flex-col"
                  action="primary"
                  onPress={() => handlePrimitiveAction("grasp", selectedObjectId)}
                >
                  <ButtonIcon as={HandIcon} />
                  <ButtonText>Grasp</ButtonText>
                </Button>
                <Button
                  className="h-full flex-1 flex-col"
                  action="primary"
                  onPress={() => handlePrimitiveAction("lift", selectedObjectId)}
                >
                  <ButtonIcon as={SlidersHorizontalIcon} />
                  <ButtonText>Lift</ButtonText>
                </Button>
                <Button
                  className="h-full flex-1 flex-col"
                  action="primary"
                  onPress={() => handlePrimitiveAction("drop", selectedObjectId)}
                >
                  <ButtonIcon as={SquareIcon} />
                  <ButtonText>Drop</ButtonText>
                </Button>
              </View>
            </View>
          )}
          
          {transcript ? <Text className="mt-2 text-sm">Heard: {transcript}</Text> : null}
        </Card>
      </View>
    </SafeAreaView>
  );
}

