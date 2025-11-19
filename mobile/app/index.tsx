import { Card } from "@/components/ui/card";
import { Heading } from "@/components/ui/heading";
import { Text } from "@/components/ui/text";
import { useRouter } from "expo-router";
import React, { useRef, useState } from "react";
import { FlatList, Pressable, View, Alert, Platform } from "react-native";
import { Audio } from "expo-av";
import { Mic as MicIcon } from "lucide-react-native";
import { BACKEND_URL } from "@/config/backend";

const mockTasks = [
  { id: "1", title: "Prepare Medicine", description: "Prepare the medicine for the patient." },
  { id: "2", title: "Set up the table", description: "Set the table for the meal." },
  { id: "3", title: "Organize books", description: "Organize the books on the shelf." },
];

export default function Home() {
  const router = useRouter();
  const [isRecording, setIsRecording] = useState(false);
  const [isUploading, setIsUploading] = useState(false);
  const recordingRef = useRef<any>(null);
  const recognitionRef = useRef<any>(null);
  // navigation will push directly to per-task routes (/task/1 etc.)

  const startSpeechRecognition = async () => {
    const globalAny: any = typeof window !== "undefined" ? window : null;
    const SpeechRecognition = globalAny?.SpeechRecognition || globalAny?.webkitSpeechRecognition;

  const navigateForTranscript = (transcript: string) => {
    const t = String(transcript).toLowerCase();
    if (t.includes("prepare medicine") || t.includes("prepare the medicine")) {
  router.push((`/task/1`) as any);
    } else if (t.includes("set up table") || t.includes("set the table") || t.includes("setup table")) {
  router.push((`/task/2`) as any);
    } else if (t.includes("organize books") || t.includes("organise books")) {
  router.push((`/task/3`) as any);
    } else {
      if (globalAny?.alert) globalAny.alert(`Heard: ${transcript}`);
    }
  };

    if (SpeechRecognition) {
      // Toggle web SpeechRecognition: start if not recording, stop if recording
      if (!isRecording) {
        const recognition = new SpeechRecognition();
        recognition.lang = "en-US";
        recognition.interimResults = false;
        recognition.maxAlternatives = 1;

        recognition.onresult = (ev: any) => {
          const transcript = String(ev.results[0][0].transcript);
          navigateForTranscript(transcript);
        };

        recognition.onerror = (e: any) => {
          if (globalAny?.alert) globalAny.alert(`Speech error: ${e?.error ?? e}`);
        };

        recognition.onend = () => {
          setIsRecording(false);
          recognitionRef.current = null;
        };

        recognitionRef.current = recognition;
        try {
          recognition.start();
          setIsRecording(true);
        } catch (e) {
          if (globalAny?.alert) globalAny.alert(String(e));
          recognitionRef.current = null;
          setIsRecording(false);
        }
      } else {
        // stop recognition
        try {
          recognitionRef.current?.stop?.();
        } catch (e) {
          /* ignore */
        }
        recognitionRef.current = null;
        setIsRecording(false);
      }

      return;
    }

    // Native path: record audio in Expo Go and upload to backend for transcription
    if (Platform.OS !== "web") {
      // Toggle recording: start if not recording, stop and upload if recording
  if (!isRecording) {
        try {
          const p = await Audio.requestPermissionsAsync();
          if (p.status !== 'granted') {
            Alert.alert('Permission required', 'Microphone permission is required to record audio.');
            return;
          }

          await Audio.setAudioModeAsync({ allowsRecordingIOS: true, playsInSilentModeIOS: true });
          const recording = new Audio.Recording();
          // Some versions of expo-av expose recording presets under different types — cast to any to avoid type mismatch
          // eslint-disable-next-line @typescript-eslint/no-explicit-any
          await recording.prepareToRecordAsync((Audio as any).RECORDING_OPTIONS_PRESET_HIGH_QUALITY || undefined);
          await recording.startAsync();
          recordingRef.current = recording;
          setIsRecording(true);
        } catch (e: any) {
          Alert.alert('Recording error', String(e));
        }
      } else {
        // Stop and upload
        try {
          const recording = recordingRef.current;
          await recording.stopAndUnloadAsync();
          const uri = recording.getURI();
          setIsRecording(false);
          recordingRef.current = null;

          if (!uri) {
            Alert.alert('Recording error', 'No audio recorded');
            return;
          }

          const formData = new FormData();
          // name and type may vary by platform
          const filename = uri.split('/').pop() || 'recording.m4a';
          formData.append('file', {
            uri,
            name: filename,
            type: 'audio/m4a',
          } as any);

          // Quick reachability check: ping backend root before uploading
          const ping = async (url: string, timeout = 5000) => {
            return await Promise.race([
              fetch(url, { method: 'GET' }),
              new Promise((_, reject) => setTimeout(() => reject(new Error('ping timeout')), timeout)),
            ]);
          };

          try {
            setIsUploading(true);
            await ping(`${BACKEND_URL}/`);
          } catch (e: any) {
            setIsUploading(false);
            Alert.alert(
              'Network Error',
              `Cannot reach backend at ${BACKEND_URL}.\n\n` +
              `Troubleshooting:\n` +
              `1. Make sure your iPad and computer are on the same Wi-Fi network\n` +
              `2. Verify the backend server is running on your computer\n` +
              `3. Check that the IP address in mobile/config/backend.ts matches your computer's IP\n` +
              `4. Try disabling firewall temporarily to test\n` +
              `5. Make sure your computer's IP hasn't changed (DHCP)\n\n` +
              `Error: ${String(e?.message ?? e)}`
            );
            return;
          }

          // Do not set Content-Type header; letting fetch set the correct multipart boundary fixes React Native network errors
          let resp;
          try {
            resp = await fetch(`${BACKEND_URL}/voice/audio`, {
              method: 'POST',
              body: formData,
            });
          } catch (e: any) {
            setIsUploading(false);
            Alert.alert('Upload error', `Network request failed when uploading to ${BACKEND_URL}/voice/audio\n${String(e?.message ?? e)}`);
            return;
          }

          if (!resp.ok) {
            const txt = await resp.text();
            Alert.alert('Transcription error', txt || String(resp.status));
            return;
          }

          const j = await resp.json();
          const transcript = j?.transcript ?? j?.result?.get('command') ?? null;
          if (transcript) {
            navigateForTranscript(transcript);
          } else {
            Alert.alert('No transcript', JSON.stringify(j));
          }
          setIsUploading(false);
        } catch (e: any) {
          Alert.alert('Upload error', String(e));
          setIsRecording(false);
          recordingRef.current = null;
          setIsUploading(false);
        }
      }

      return;
    }

    if (globalAny?.alert) globalAny.alert("Speech recognition not available in this browser");
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
      </View>
    </View>
  );
}
