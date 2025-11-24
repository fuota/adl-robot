import { Card } from "@/components/ui/card";
import { Heading } from "@/components/ui/heading";
import { Text } from "@/components/ui/text";
import { useRouter } from "expo-router";
import { Mic as MicIcon } from "lucide-react-native";
import { useEffect, useRef, useState } from "react";
import { Alert, FlatList, Platform, Pressable, View } from "react-native";

// Import Voice module
let Voice: any = null;
try {
  // eslint-disable-next-line @typescript-eslint/no-var-requires
  const voiceModule = require("@react-native-voice/voice");
  Voice = voiceModule.default || voiceModule;
} catch (e) {
  Voice = null;
}

export default function Home() {
  const router = useRouter();
  const { tasks, isLoading } = useTask();
  const [isRecording, setIsRecording] = useState(false);
  const recognitionRef = useRef<any>(null);
  const nativeListeningRef = useRef(false);

  // Helper function to stop speech recognition
  const stopSpeechRecognition = async () => {
    if (Voice && Platform.OS !== "web" && nativeListeningRef.current) {
      try {
        if (typeof Voice.stop === "function") {
          await Voice.stop();
        }
      } catch (e) {
        console.error("Error stopping voice:", e);
      }
      nativeListeningRef.current = false;
      setIsRecording(false);
    }
  };

  // Store the latest transcript
  const latestTranscriptRef = useRef<string>("");

  // Set up Voice event listeners
  useEffect(() => {
    if (Voice && Platform.OS !== "web") {
      // Handle partial results (interim results while speaking)
      const onSpeechPartialResults = (event: any) => {
        const text = (event?.value && event.value[0]) || "";
        if (text) {
          latestTranscriptRef.current = text;
          console.log("Partial result:", text);
        }
      };

      // Handle final results (complete phrase)
      const onSpeechResults = async (event: any) => {
        const text =
          (event?.value && event.value[0]) || latestTranscriptRef.current || "";
        console.log("Final result:", text);
        // Stop recognition first
        await stopSpeechRecognition();
        // Clear the partial results
        latestTranscriptRef.current = "";
        // Check if we got any text
        if (!text || text.trim().length === 0) {
          Alert.alert(
            "No Speech Detected",
            'Please speak a command. Try saying:\n- "Prepare medicine"\n- "Set up table"\n- "Organize books"',
          );
          return;
        }
        // Then process the transcript
        navigateForTranscript(text);
      };

      const onSpeechError = async (event: any) => {
        await stopSpeechRecognition();
        latestTranscriptRef.current = "";

        const errorCode = event?.error?.code || event?.code;
        const errorMessage =
          event?.error?.message || event?.message || "Unknown error";

        // Handle specific error cases
        if (
          errorCode === 6 ||
          errorMessage.includes("no speech") ||
          errorMessage.includes("no match")
        ) {
          Alert.alert(
            "No Speech Detected",
            "No speech was detected. Please try again and speak clearly.",
          );
        } else if (
          errorCode === 7 ||
          errorMessage.includes("recognition not available")
        ) {
          Alert.alert(
            "Speech Recognition Unavailable",
            "Speech recognition is not available on this device.",
          );
        } else {
          Alert.alert("Speech Error", `Please try again and speak clearly.`);
        }
      };

      // Handle when speech ends - check if we got any results
      const onSpeechEnd = async () => {
        console.log("Speech ended, waiting for final results...");
        // Set a timeout to handle case where no results come after speech ends
        setTimeout(async () => {
          // If we still have no transcript after a delay, it means no speech was detected
          if (!latestTranscriptRef.current && isRecording) {
            await stopSpeechRecognition();
            Alert.alert(
              "No Speech Detected",
              "No speech was detected. Please try again and speak clearly.",
            );
          }
        }, 2000); // Wait 2 seconds for results
      };

      Voice.onSpeechPartialResults = onSpeechPartialResults;
      Voice.onSpeechResults = onSpeechResults;
      Voice.onSpeechError = onSpeechError;
      Voice.onSpeechEnd = onSpeechEnd;

      return () => {
        // Cleanup
        stopSpeechRecognition();
        if (Voice && typeof Voice.destroy === "function") {
          Voice.destroy().catch(() => {});
        }
        if (Voice && typeof Voice.removeAllListeners === "function") {
          Voice.removeAllListeners();
        }
      };
    }
  }, []);

  const navigateForTranscript = async (transcript: string) => {
    // Ensure speech recognition is stopped before processing
    await stopSpeechRecognition();

    const t = String(transcript).toLowerCase().trim();

    // Task 1: Prepare medicine - match "prepare", "prep", "medicine", or full phrase
    if (
      t.includes("medicine") ||
      t.startsWith("prep") ||
      (t.includes("prep") && t.includes("med")) ||
      t === "medicine" ||
      t === "med"
    ) {
      router.push(`/task/1` as any);
    }
    // Task 2: Set up table - match "set", "table", "setup", or full phrase
    else if (
      t.includes("set up table") ||
      t.includes("set the table") ||
      t.includes("setup table") ||
      t.startsWith("set") ||
      t === "table" ||
      t.includes("table")
    ) {
      router.push(`/task/2` as any);
    }
    // Task 3: Organize books - match "organize", "organise", "book", "books", or full phrase
    else if (
      t.includes("book") ||
      t.includes("organize") ||
      t.includes("organise")
    ) {
      router.push(`/task/3` as any);
    } else {
      Alert.alert(
        "Command not recognized",
        `Heard: "${transcript}"\n\nTry saying:\n- "Prepare medicine" or "prep" or "medicine"\n- "Set up table" or "set" or "table"\n- "Organize books" or "organize" or "books"`,
      );
    }
  };

  const startSpeechRecognition = async () => {
    const globalAny: any = typeof window !== "undefined" ? window : null;

    // 1. Try native voice recognition first (iOS/Android native builds)
    if (Voice && Platform.OS !== "web") {
      if (!isRecording) {
        try {
          // Check if Voice has the start method
          if (!Voice || typeof Voice.start !== "function") {
            Alert.alert(
              "Speech Error",
              "Voice recognition module not properly initialized. Please rebuild the app.",
            );
            console.error("Voice module:", Voice);
            return;
          }

          // Stop any existing recognition first to avoid "already started" error
          if (
            nativeListeningRef.current &&
            Voice &&
            typeof Voice.stop === "function"
          ) {
            try {
              await Voice.stop();
            } catch (stopError) {
              // Ignore stop errors, just continue
              console.log("Stop error (ignored):", stopError);
            }
            nativeListeningRef.current = false;
          }

          // Small delay to ensure previous recognition is fully stopped
          await new Promise((resolve) => setTimeout(resolve, 100));

          Voice.start("en-US");
          setIsRecording(true);
          nativeListeningRef.current = true;
        } catch (e: any) {
          console.error("Voice start error:", e);
          setIsRecording(false);
          nativeListeningRef.current = false;
          Alert.alert(
            "Speech Error",
            `Failed to start speech recognition: ${String(e?.message || e)}`,
          );
        }
      } else {
        // Stop native recognition
        await stopSpeechRecognition();
      }
      return;
    }

    // 2. Try web SpeechRecognition API (works in web browsers)
    const SpeechRecognition =
      globalAny?.SpeechRecognition || globalAny?.webkitSpeechRecognition;
    if (SpeechRecognition) {
      if (!isRecording) {
        try {
          const recognition = new SpeechRecognition();
          recognition.lang = "en-US";
          recognition.interimResults = false;
          recognition.maxAlternatives = 1;
          recognition.continuous = false;

          recognition.onresult = (ev: any) => {
            const transcript = String(ev.results[0][0].transcript);
            navigateForTranscript(transcript);
          };

          recognition.onerror = (e: any) => {
            console.error("Speech recognition error", e);
            setIsRecording(false);
            Alert.alert(
              "Speech Error",
              `Could not recognize speech: ${e?.error || "Unknown error"}`,
            );
          };

          recognition.onend = () => {
            setIsRecording(false);
            recognitionRef.current = null;
          };

          recognitionRef.current = recognition;
          recognition.start();
          setIsRecording(true);
        } catch (e: any) {
          Alert.alert(
            "Speech Error",
            `Failed to start speech recognition: ${String(e)}`,
          );
          setIsRecording(false);
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
        setIsRecording(false);
      }
      return;
    }

    // 3. Fallback: Show helpful message
    if (Platform.OS === "web") {
      Alert.alert(
        "Speech Recognition Not Available",
        "Your browser doesn't support speech recognition. Please use Chrome, Edge, or Safari.",
      );
    } else {
      Alert.alert(
        "Speech Recognition Not Available",
        "Native speech recognition requires a custom development build.\n\n" +
          "Make sure you're running a native build (not Expo Go) with @react-native-voice/voice installed.",
      );
    }
  };

  return (
    <View className="py-safe flex-1 bg-gray-50 px-5">
      <Heading size="2xl" className="mb-6">
        ADL Robot
      </Heading>

      <Heading className="mb-4" size="xl">
        Available Tasks
      </Heading>
      {isLoading ? (
        <View className="items-center justify-center py-8">
          <Text>Loading tasks...</Text>
        </View>
      ) : (
        <FlatList
          data={tasks}
          keyExtractor={(item) => item.id}
          ItemSeparatorComponent={() => <View className="h-3" />}
          renderItem={({ item }) => (
            <Pressable
              onPress={() => {
                // Navigate directly to per-task route
                router.push(`/task/${item.id}` as any);
              }}
            >
              <Card variant="outline" className="text-center">
                <Heading>{item.title}</Heading>
                <Text>{item.description}</Text>
                <Text className="mt-1 text-xs text-gray-500">
                  {item.totalSteps} steps • ~
                  {Math.round(item.estimatedDuration / 60)} min
                </Text>
              </Card>
            </Pressable>
          )}
        />
      )}

      {/* Speech button under tasks */}
      <View className="mt-6 items-center">
        <Pressable
          onPress={startSpeechRecognition}
          className={`h-28 w-28 rounded-full ${isRecording ? "bg-red-500" : "bg-blue-500"} items-center justify-center`}
          accessibilityLabel={
            isRecording ? "Stop listening" : "Start speech command"
          }
        >
          <View>
            <MicIcon color="white" size={36} />
          </View>
        </Pressable>

        <Text className="mt-3 text-sm text-gray-900">
          {isRecording ? "Listening... tap to stop" : "Speak a command"}
        </Text>
        <Text className="mt-2 text-center text-xs text-gray-500">
          Try: "Prepare medicine", "Set up table", or "Organize books"
        </Text>
      </View>
    </View>
  );
}
