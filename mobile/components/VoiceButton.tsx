import React, { useEffect, useRef, useState } from "react";
import { ActivityIndicator, Alert, Platform, Pressable, Text, View } from "react-native";
import { Mic as MicIcon } from "lucide-react-native";

type Props = {
  onResult: (text: string) => void;
  onError?: (err: string) => void;
};

export default function VoiceButton({ onResult, onError }: Props) {
  const [listening, setListening] = useState(false);
  const recognitionRef = useRef<any>(null);

  useEffect(() => {
    // Only wire up web SpeechRecognition here; native platforms need native modules.
    const globalAny: any = typeof window !== "undefined" ? window : global;
    const SpeechRecognition =
      globalAny.SpeechRecognition || globalAny.webkitSpeechRecognition;

    if (SpeechRecognition) {
      const recognition = new SpeechRecognition();
      recognition.lang = "en-US";
      recognition.interimResults = false;
      recognition.maxAlternatives = 1;

      recognition.onresult = (event: any) => {
        const transcript = event.results[0][0].transcript as string;
        setListening(false);
        onResult(transcript);
      };

      recognition.onerror = (event: any) => {
        setListening(false);
        const message = event?.error || "Speech recognition error";
        onError?.(message);
      };

      recognitionRef.current = recognition;
    }
  }, []);

  const startListening = () => {
    // Web: use SpeechRecognition if available
    const globalAny: any = typeof window !== "undefined" ? window : global;
    const SpeechRecognition =
      globalAny.SpeechRecognition || globalAny.webkitSpeechRecognition;

    if (SpeechRecognition && recognitionRef.current) {
      try {
        (recognitionRef.current as any).start();
        setListening(true);
      } catch (err: any) {
        setListening(false);
        onError?.(String(err));
      }
      return;
    }

    // Fallback for native: show an alert informing the user and ask to type the command.
    if (Platform.OS === "android" || Platform.OS === "ios") {
      Alert.prompt
        ? Alert.prompt(
            "Voice command",
            "Speech recognition is not available in this build. Type your command:",
            (text) => {
              if (text) onResult(text);
            }
          )
        : Alert.alert(
            "Voice command",
            "Speech recognition is not available. Please type the command in the input field (not supported on this platform).",
            [{ text: "OK" }]
          );
      return;
    }

    onError?.("Speech recognition not supported in this environment");
  };

  return (
    <Pressable
      onPress={startListening}
      className="h-16 w-16 rounded-full bg-blue-600 items-center justify-center"
      accessibilityLabel="Voice command"
    >
      <View className="items-center justify-center">
        {listening ? (
          <ActivityIndicator color="white" />
        ) : (
          <MicIcon color="white" width={28} height={28} />
        )}
      </View>
    </Pressable>
  );
}
