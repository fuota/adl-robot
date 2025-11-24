import type {
  AvailableTasksResponse,
  Task,
  TaskDefinition,
  TaskProgress,
} from "@/types/tasks";
import React, {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useState,
} from "react";
import { useRos } from "./RosContext";

type TaskContextType = {
  selectedId: string | null;
  setSelectedId: (id: string | null) => void;
  tasks: Task[];
  currentProgress: TaskProgress | null;
  loadTasks: () => void;
  startTask: (taskId: string) => void;
  isLoading: boolean;
};

const TaskContext = createContext<TaskContextType | undefined>(undefined);

// Map task IDs to ROS service names
const TASK_SERVICE_MAP: { [key: string]: string } = {
  "1": "prepare_medicine",
  "2": "setup_tableware",
  "3": "organize_books",
};

export const TaskProvider = ({ children }: { children: React.ReactNode }) => {
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [tasks, setTasks] = useState<Task[]>([]);
  const [currentProgress, setCurrentProgress] = useState<TaskProgress | null>(
    null,
  );
  const [isLoading, setIsLoading] = useState(false);
  const { ros, connected } = useRos();

  // Convert ROS task definition to UI task format
  const convertToUITask = (
    taskId: string,
    taskKey: string,
    taskDef: TaskDefinition,
  ): Task => {
    return {
      id: taskId,
      title: taskDef.name,
      description: taskDef.description,
      progress: 0,
      currentStep: taskDef.steps[0]?.description || "Ready",
      status: "not-started",
      steps: taskDef.steps.map((step) => ({
        title: step.description,
        status: "not-started" as const,
      })),
      totalSteps: taskDef.total_steps,
      estimatedDuration: taskDef.estimated_duration_seconds,
    };
  };

  // Load available tasks from ROS
  const loadTasks = useCallback(() => {
    if (!connected || typeof ros?.callService !== "function") {
      // Set default tasks when ROS is not available
      setTasks([
        {
          id: "1",
          title: "Prepare Medicine",
          description: "Places water cup, plate, and pours water and medicine",
          progress: 0,
          currentStep: "Ready",
          status: "not-started",
          steps: [
            { title: "Place water cup in serving area", status: "not-started" },
            { title: "Place plate in serving area", status: "not-started" },
            { title: "Pour water into cup", status: "not-started" },
            { title: "Pour medicine into plate", status: "not-started" },
          ],
          totalSteps: 4,
          estimatedDuration: 120,
        },
        {
          id: "2",
          title: "Setup Tableware",
          description: "Places bowl, fork, and spoon in serving areas",
          progress: 0,
          currentStep: "Ready",
          status: "not-started",
          steps: [
            { title: "Place bowl in serving area", status: "not-started" },
            { title: "Place fork in serving area", status: "not-started" },
            { title: "Place spoon in serving area", status: "not-started" },
          ],
          totalSteps: 3,
          estimatedDuration: 90,
        },
        {
          id: "3",
          title: "Organize Books",
          description: "Places books in bookshelf compartments",
          progress: 0,
          currentStep: "Ready",
          status: "not-started",
          steps: [
            { title: "Place book 1 in compartment 1", status: "not-started" },
            { title: "Place book 2 in compartment 2", status: "not-started" },
          ],
          totalSteps: 2,
          estimatedDuration: 60,
        },
      ]);
      return;
    }

    setIsLoading(true);
    try {
      // eslint-disable-next-line @typescript-eslint/no-var-requires
      const ROSLIB = require("roslib");
      const getTasksService = new ROSLIB.Service({
        ros: ros,
        name: "/get_available_tasks",
        serviceType: "std_srvs/Trigger",
      });

      const request = new ROSLIB.ServiceRequest({});

      getTasksService.callService(
        request,
        (result: any) => {
          try {
            const response: AvailableTasksResponse = JSON.parse(result.message);
            const loadedTasks: Task[] = [];

            // Map ROS task keys to UI task IDs
            const taskKeyToId: { [key: string]: string } = {
              prepare_medicine: "1",
              setup_tableware: "2",
              organize_books: "3",
            };

            Object.entries(response.tasks).forEach(([taskKey, taskDef]) => {
              const taskId = taskKeyToId[taskKey];
              if (taskId) {
                loadedTasks.push(convertToUITask(taskId, taskKey, taskDef));
              }
            });

            // Sort by ID to maintain consistent order
            loadedTasks.sort((a, b) => a.id.localeCompare(b.id));
            setTasks(loadedTasks);
          } catch (e) {
            console.error("Failed to parse tasks response:", e);
          } finally {
            setIsLoading(false);
          }
        },
        (error: any) => {
          console.error("Failed to load tasks:", error);
          setIsLoading(false);
        },
      );
    } catch (e) {
      console.error("Error calling get_available_tasks:", e);
      setIsLoading(false);
    }
  }, [connected, ros]);

  // Subscribe to task progress updates
  useEffect(() => {
    if (!connected || typeof ros?.subscribe !== "function") return;

    try {
      // eslint-disable-next-line @typescript-eslint/no-var-requires
      const ROSLIB = require("roslib");
      const progressTopic = new ROSLIB.Topic({
        ros: ros,
        name: "/task_progress",
        messageType: "std_msgs/String",
      });

      progressTopic.subscribe((message: any) => {
        try {
          const progress: TaskProgress = JSON.parse(message.data);
          setCurrentProgress(progress);

          // Update task state based on progress
          setTasks((prevTasks) =>
            prevTasks.map((task) => {
              const taskKey = TASK_SERVICE_MAP[task.id];
              if (taskKey === progress.task) {
                // Update steps status
                const updatedSteps = task.steps.map((step, index) => {
                  const stepNumber = index + 1;
                  let status:
                    | "not-started"
                    | "in-progress"
                    | "completed"
                    | "failed" = "not-started";

                  if (stepNumber < progress.current_step) {
                    status = "completed";
                  } else if (stepNumber === progress.current_step) {
                    status =
                      progress.status === "failed" ? "failed" : "in-progress";
                  }

                  return { ...step, status };
                });

                return {
                  ...task,
                  progress: progress.progress_percent,
                  currentStep: progress.description,
                  status: progress.status,
                  steps: updatedSteps,
                };
              }
              return task;
            }),
          );
        } catch (e) {
          console.error("Failed to parse progress update:", e);
        }
      });

      return () => {
        try {
          progressTopic.unsubscribe();
        } catch (e) {
          console.error("Error unsubscribing from progress topic:", e);
        }
      };
    } catch (e) {
      console.error("Error subscribing to task_progress:", e);
    }
  }, [connected, ros]);

  // Load tasks when connected
  useEffect(() => {
    if (connected) {
      loadTasks();
    }
  }, [connected, loadTasks]);

  // Start a task by calling the appropriate ROS service
  const startTask = useCallback(
    (taskId: string) => {
      const serviceName = TASK_SERVICE_MAP[taskId];
      if (
        !serviceName ||
        !connected ||
        typeof ros?.callService !== "function"
      ) {
        console.error(
          "Cannot start task: ROS not connected or invalid task ID",
        );
        return;
      }

      try {
        // eslint-disable-next-line @typescript-eslint/no-var-requires
        const ROSLIB = require("roslib");
        const taskService = new ROSLIB.Service({
          ros: ros,
          name: `/${serviceName}`,
          serviceType: "std_srvs/Trigger",
        });

        const request = new ROSLIB.ServiceRequest({});

        // Update task to in-progress immediately
        setTasks((prevTasks) =>
          prevTasks.map((task) =>
            task.id === taskId
              ? { ...task, status: "in-progress" as const, progress: 0 }
              : task,
          ),
        );

        taskService.callService(
          request,
          (result: any) => {
            console.log(
              `Task ${serviceName} completed:`,
              result.success ? "Success" : "Failed",
            );
            if (!result.success) {
              console.error("Task failed:", result.message);
            }
          },
          (error: any) => {
            console.error(`Failed to call ${serviceName}:`, error);
            // Reset task status on error
            setTasks((prevTasks) =>
              prevTasks.map((task) =>
                task.id === taskId
                  ? { ...task, status: "failed" as const }
                  : task,
              ),
            );
          },
        );
      } catch (e) {
        console.error(`Error calling ${serviceName}:`, e);
      }
    },
    [connected, ros],
  );

  return (
    <TaskContext.Provider
      value={{
        selectedId,
        setSelectedId,
        tasks,
        currentProgress,
        loadTasks,
        startTask,
        isLoading,
      }}
    >
      {children}
    </TaskContext.Provider>
  );
};

export const useTask = () => {
  const ctx = useContext(TaskContext);
  if (!ctx) throw new Error("useTask must be used within TaskProvider");
  return ctx;
};

export default TaskContext;
