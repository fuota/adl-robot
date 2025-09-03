import { Card } from "@/components/ui/card";
import { Heading } from "@/components/ui/heading";
import { Text } from "@/components/ui/text";
import { useRouter } from "expo-router";
import React from "react";
import { FlatList, Pressable, View } from "react-native";

const mockTasks = [
  { id: "1", title: "Task 1", description: "Description for Task 1" },
  { id: "2", title: "Task 2", description: "Description for Task 2" },
  { id: "3", title: "Task 3", description: "Description for Task 3" },
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
