import { Card } from "@/components/ui/card";
import { Heading } from "@/components/ui/heading";
import { Text } from "@/components/ui/text";
import { useRouter } from "expo-router";
import React from "react";
import { FlatList, Pressable, View } from "react-native";

const mockTasks = [
  { id: "1", title: "Prepare Medicine", description: "Prepare the medicine for the patient." },
  { id: "2", title: "Set up the table", description: "Set the table for the meal." },
  { id: "3", title: "Organize books", description: "Organize the books on the shelf." },
];

export default function Home() {
  const router = useRouter();

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
              router.push("/task");
            }}
          >
            <Card variant="outline" className="text-center">
              <Heading>{item.title}</Heading>
              <Text>{item.description}</Text>
            </Card>
          </Pressable>
        )}
      />
    </View>
  );
}
