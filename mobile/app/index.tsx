import { Card } from "@/components/ui/card";
import { Heading } from "@/components/ui/heading";
import { Text } from "@/components/ui/text";
import { useRouter } from "expo-router";
import React, { useRef, useState, useEffect } from "react";
import { FlatList, Pressable, View, Alert, Platform } from "react-native";
import { Mic as MicIcon } from "lucide-react-native";

// Import Voice module
let Voice: any = null;
try {
  // eslint-disable-next-line @typescript-eslint/no-var-requires
  const voiceModule = require("@react-native-voice/voice");
  Voice = voiceModule.default || voiceModule;
} catch (e) {
  Voice = null;
}

const mockTasks = [
  { id: "1", title: "Set up the table", description: "Set the table for the meal." },
  { id: "2", title: "Prepare Medicine", description: "Prepare the medicine for the patient." },
  { id: "3", title: "Organize books", description: "Organize the books on the shelf." },
];

export default function Home() {
  const router = useRouter();
  const [isRecording, setIsRecording] = useState(false);
  const recognitionRef = useRef<any>(null);
  const nativeListeningRef = useRef(false);

  // Helper function to stop speech recognition
  const stopSpeechRecognition = async () => {
    // Clear any pending timeouts
    if (processingTimeoutRef.current) {
      clearTimeout(processingTimeoutRef.current);
      processingTimeoutRef.current = null;
    }
    if (silenceTimeoutRef.current) {
      clearTimeout(silenceTimeoutRef.current);
      silenceTimeoutRef.current = null;
    }
    
    if (Voice && Platform.OS !== "web" && nativeListeningRef.current) {
      try {
        if (typeof Voice.stop === 'function') {
          await Voice.stop();
        }
      } catch (e) {
        console.error("Error stopping voice:", e);
      }
      nativeListeningRef.current = false;
      setIsRecording(false);
    }
  };

  // Store the latest transcript and debounce timer
  const latestTranscriptRef = useRef<string>("");
  const processingTimeoutRef = useRef<any>(null);
  const silenceTimeoutRef = useRef<any>(null);

  // Set up Voice event listeners
  useEffect(() => {
    if (Voice && Platform.OS !== "web") {
      // Handle partial results (interim results while speaking)
      // Accumulate partial results and wait for silence before processing
      const onSpeechPartialResults = (event: any) => {
        const text = (event?.value && event.value[0]) || "";
        if (text) {
          latestTranscriptRef.current = text;
          console.log("Partial result (accumulating):", text);
          
          // Clear any existing silence timeout
          if (silenceTimeoutRef.current) {
            clearTimeout(silenceTimeoutRef.current);
          }
          
          // Wait for 2 seconds of silence before processing
          // This ensures we capture the complete phrase
          silenceTimeoutRef.current = setTimeout(() => {
            if (latestTranscriptRef.current && latestTranscriptRef.current.trim().length > 0) {
              console.log("Silence detected, processing accumulated text:", latestTranscriptRef.current);
              const textToProcess = latestTranscriptRef.current;
              latestTranscriptRef.current = "";
              
              // Clear processing timeout since we're processing now
              if (processingTimeoutRef.current) {
                clearTimeout(processingTimeoutRef.current);
                processingTimeoutRef.current = null;
              }
              
              stopSpeechRecognition().then(() => {
                navigateForTranscript(textToProcess);
              });
            }
          }, 2000); // Wait 2 seconds of silence
        }
      };

      // Handle final results (complete phrase)
      // Don't process immediately - just update transcript and wait for more or silence
      const onSpeechResults = async (event: any) => {
        const text = (event?.value && event.value[0]) || "";
        console.log("Final result received (not processing yet):", text);
        
        // Update the latest transcript with the new result
        if (text) {
          // Append to existing if we have partial results, or replace
          if (latestTranscriptRef.current && !latestTranscriptRef.current.includes(text)) {
            latestTranscriptRef.current = latestTranscriptRef.current + " " + text;
          } else {
            latestTranscriptRef.current = text;
          }
        }
        
        // Don't process immediately - wait for silence or more results
        // Clear any existing processing timeout
        if (processingTimeoutRef.current) {
          clearTimeout(processingTimeoutRef.current);
        }
        
        // Set a longer delay (3 seconds) to wait for more words
        // This helps capture longer phrases like "set up table"
        processingTimeoutRef.current = setTimeout(async () => {
          const finalText = latestTranscriptRef.current || text;
          console.log("Processing final text after delay:", finalText);
          
          // Stop recognition
        await stopSpeechRecognition();
          
          // Clear refs
        latestTranscriptRef.current = "";
          if (processingTimeoutRef.current) {
            clearTimeout(processingTimeoutRef.current);
            processingTimeoutRef.current = null;
          }
          if (silenceTimeoutRef.current) {
            clearTimeout(silenceTimeoutRef.current);
            silenceTimeoutRef.current = null;
          }
          
        // Check if we got any text
          if (!finalText || finalText.trim().length === 0) {
          Alert.alert("No Speech Detected", "Please speak a command. Try saying:\n- \"Set up table\"\n- \"Prepare medicine\"\n- \"Organize books\"");
          return;
        }
        // Then process the transcript
          navigateForTranscript(finalText);
        }, 3000); // Wait 3 seconds to ensure complete phrase is captured
      };

      const onSpeechError = async (event: any) => {
        await stopSpeechRecognition();
        latestTranscriptRef.current = "";

        const errorCode = event?.error?.code || event?.code;
        const errorMessage = event?.error?.message || event?.message || "Unknown error";

        // Handle specific error cases
        if (errorCode === 6 || errorMessage.includes("no speech") || errorMessage.includes("no match")) {
          Alert.alert("No Speech Detected", "No speech was detected. Please try again and speak clearly.");
        } else if (errorCode === 7 || errorMessage.includes("recognition not available")) {
          Alert.alert("Speech Recognition Unavailable", "Speech recognition is not available on this device.");
        } else {
          Alert.alert("Speech Error", `Please try again and speak clearly.`);
        }
      
      };

      // Handle when speech ends - check if we got any results
      const onSpeechEnd = async () => {
        console.log("Speech ended, waiting for final results...");
        // Set a longer timeout to handle case where no results come after speech ends
        // This gives more time for complete phrases to be processed
        setTimeout(async () => {
          // If we have accumulated text, process it
          if (latestTranscriptRef.current && latestTranscriptRef.current.trim().length > 0) {
            const textToProcess = latestTranscriptRef.current;
            latestTranscriptRef.current = "";
            await stopSpeechRecognition();
            navigateForTranscript(textToProcess);
          } else if (!latestTranscriptRef.current && isRecording) {
            await stopSpeechRecognition();
            Alert.alert("No Speech Detected", "No speech was detected. Please try again and speak clearly.");
          }
        }, 3000); // Wait 3 seconds for results (increased from 2s to capture longer phrases)
      };

      Voice.onSpeechPartialResults = onSpeechPartialResults;
      Voice.onSpeechResults = onSpeechResults;
      Voice.onSpeechError = onSpeechError;
      Voice.onSpeechEnd = onSpeechEnd;

      return () => {
        // Cleanup
        stopSpeechRecognition();
        if (processingTimeoutRef.current) {
          clearTimeout(processingTimeoutRef.current);
          processingTimeoutRef.current = null;
        }
        if (silenceTimeoutRef.current) {
          clearTimeout(silenceTimeoutRef.current);
          silenceTimeoutRef.current = null;
        }
        if (Voice && typeof Voice.destroy === 'function') {
          Voice.destroy().catch(() => {});
        }
        if (Voice && typeof Voice.removeAllListeners === 'function') {
          Voice.removeAllListeners();
        }
      };
    }
  }, []);

  const navigateForTranscript = async (transcript: string) => {
    // Ensure speech recognition is stopped before processing
    await stopSpeechRecognition();

    const t = String(transcript).toLowerCase().trim();

    // Task 1: Set up table - require full phrases only
    if (
      t.includes("set up table") ||
      t.includes("set the table") ||
      t.includes("setup table") ||
      t === "set up table" ||
      t === "set the table"
    ) {
      router.push((`/task/1`) as any);
    } 
    // Task 2: Prepare medicine - require full phrases only
    else if (
      t.includes("prepare medicine") || 
      t.includes("prepare the medicine") ||
      t === "prepare medicine" ||
      t === "prepare the medicine"
    ) {
      router.push((`/task/2`) as any);
    }
    // Task 3: Organize books - require full phrases only
    else if (
      t.includes("organize books") || 
      t.includes("organise books") ||
      t.includes("organize the books") ||
      t.includes("organise the books") ||
      t === "organize books" ||
      t === "organise books"
    ) {
      router.push((`/task/3`) as any);
    } 
    else {
      Alert.alert("Command not recognized", `Heard: "${transcript}"\n\nTry saying the complete phrase:\n- "Set up table"\n- "Prepare medicine"\n- "Organize books"`);
    }
  };

  const startSpeechRecognition = async () => {
    const globalAny: any = typeof window !== "undefined" ? window : null;

    // 1. Try native voice recognition first (iOS/Android native builds)
    if (Voice && Platform.OS !== "web") {
      if (!isRecording) {
        try {
          // Check if Voice has the start method
          if (!Voice || typeof Voice.start !== 'function') {
            Alert.alert("Speech Error", "Voice recognition module not properly initialized. Please rebuild the app.");
            console.error("Voice module:", Voice);
            return;
          }

          // Stop any existing recognition first to avoid "already started" error
          if (nativeListeningRef.current && Voice && typeof Voice.stop === 'function') {
            try {
              await Voice.stop();
            } catch (stopError) {
              // Ignore stop errors, just continue
              console.log("Stop error (ignored):", stopError);
            }
            nativeListeningRef.current = false;
          }

          // Small delay to ensure previous recognition is fully stopped
          await new Promise(resolve => setTimeout(resolve, 100));

          // Clear any existing timeouts
          if (processingTimeoutRef.current) {
            clearTimeout(processingTimeoutRef.current);
            processingTimeoutRef.current = null;
          }
          if (silenceTimeoutRef.current) {
            clearTimeout(silenceTimeoutRef.current);
            silenceTimeoutRef.current = null;
          }
          latestTranscriptRef.current = "";

          Voice.start("en-US");
          setIsRecording(true);
          nativeListeningRef.current = true;
          
          // Set a maximum listening time (10 seconds) to prevent indefinite listening
          setTimeout(async () => {
            if (nativeListeningRef.current && latestTranscriptRef.current) {
              const textToProcess = latestTranscriptRef.current;
              latestTranscriptRef.current = "";
              await stopSpeechRecognition();
              if (textToProcess.trim().length > 0) {
                navigateForTranscript(textToProcess);
              }
            }
          }, 10000); // Maximum 10 seconds of listening
        } catch (e: any) {
          console.error("Voice start error:", e);
          setIsRecording(false);
          nativeListeningRef.current = false;
          Alert.alert("Speech Error", `Failed to start speech recognition: ${String(e?.message || e)}`);
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
      if (!isRecording) {
        try {
          const recognition = new SpeechRecognition();
          recognition.lang = "en-US";
          recognition.interimResults = true; // Enable interim results to see partial phrases
          recognition.maxAlternatives = 1;
          recognition.continuous = true; // Keep listening until manually stopped
          
          // Store the latest complete transcript
          let capturedFinalTranscript = "";

          recognition.onresult = (ev: any) => {
            // Get the most complete transcript from all results
            let interimTranscript = "";
            let finalTranscript = "";
            
            for (let i = 0; i < ev.results.length; i++) {
              const transcript = ev.results[i][0].transcript;
              if (ev.results[i].isFinal) {
                finalTranscript += transcript + " ";
              } else {
                interimTranscript += transcript;
              }
            }
            
            // Store final transcript for timeout handler
            if (finalTranscript.trim()) {
              capturedFinalTranscript = finalTranscript.trim();
            }
            
            // Only process if we have a complete phrase (wait for final results)
            if (finalTranscript.trim()) {
              // Stop recognition and process the complete phrase
              recognition.stop();
              navigateForTranscript(finalTranscript.trim());
            }
          };

          recognition.onerror = (e: any) => {
            console.error("Speech recognition error", e);
            setIsRecording(false);
            recognition.stop();
            Alert.alert("Speech Error", `Could not recognize speech: ${e?.error || "Unknown error"}`);
          };
          
          // Add a timeout to stop listening after a reasonable time
          // This prevents it from listening indefinitely
          const recognitionTimeout = setTimeout(() => {
            if (recognitionRef.current) {
              recognition.stop();
              if (capturedFinalTranscript) {
                navigateForTranscript(capturedFinalTranscript);
              } else {
                setIsRecording(false);
                Alert.alert("No Speech Detected", "Please speak a complete command. Try saying:\n- \"Set up table\"\n- \"Prepare medicine\"\n- \"Organize books\"");
              }
            }
          }, 5000); // Listen for up to 5 seconds

          recognition.onend = () => {
            clearTimeout(recognitionTimeout);
            setIsRecording(false);
            recognitionRef.current = null;
          };

          recognitionRef.current = recognition;
          recognition.start();
          setIsRecording(true);
        } catch (e: any) {
          Alert.alert("Speech Error", `Failed to start speech recognition: ${String(e)}`);
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
        "Your browser doesn't support speech recognition. Please use Chrome, Edge, or Safari."
      );
    } else {
      Alert.alert(
        "Speech Recognition Not Available",
        "Native speech recognition requires a custom development build.\n\n" +
        "Make sure you're running a native build (not Expo Go) with @react-native-voice/voice installed."
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
        <FlatList
        data={mockTasks}
          keyExtractor={(item) => item.id}
          ItemSeparatorComponent={() => <View className="h-3" />}
          renderItem={({ item }) => (
            <Pressable
              onPress={() => {
                // Navigate directly to per-task route
              router.push((`/task/${item.id}`) as any);
              }}
            >
              <Card variant="outline" className="text-center">
                <Heading>{item.title}</Heading>
                <Text>{item.description}</Text>
              </Card>
            </Pressable>
          )}
        />

      {/* Speech button under tasks */}
      <View className="mt-6 items-center">
        <Pressable
          onPress={startSpeechRecognition}
          className={`h-28 w-28 rounded-full ${isRecording ? 'bg-red-500' : 'bg-blue-500'} items-center justify-center`}
          accessibilityLabel={isRecording ? 'Stop listening' : 'Start speech command'}
        >
          <View>
            <MicIcon color="white" size={36} />
          </View>
        </Pressable>

        <Text className="mt-3 text-sm text-gray-900">
          {isRecording ? 'Listening... tap to stop' : 'Speak a command'}
        </Text>
        <Text className="mt-2 text-xs text-gray-500 text-center">
          Try: "Set up table", "Prepare medicine", or "Organize books"
        </Text>
      </View>
    </View>
  );
}
