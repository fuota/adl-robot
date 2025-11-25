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
  stopTask: (taskId: string) => void;
  pauseTask: (taskId: string) => void;
  resumeTask: (taskId: string) => void;
  isPaused: (taskId: string) => boolean;
  isLoading: boolean;
};

const TaskContext = createContext<TaskContextType | undefined>(undefined);

// Map task IDs to ROS service names
const TASK_SERVICE_MAP: { [key: string]: string } = {
  "1": "set_up_table",      // Task 1 = Set up the table
  "2": "prepare_medicine",  // Task 2 = Prepare medicine
  "3": "organize_books",    // Task 3 = Organize books
};

export const TaskProvider = ({ children }: { children: React.ReactNode }) => {
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [tasks, setTasks] = useState<Task[]>([]);
  const [currentProgress, setCurrentProgress] = useState<TaskProgress | null>(
    null,
  );
  const [isLoading, setIsLoading] = useState(false);
  const [pausedTasks, setPausedTasks] = useState<Set<string>>(new Set());
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
    if (!connected || !ros) {
      // Set default tasks when ROS is not available
      setTasks([
        {
          id: "1",
          title: "Set Up Table",
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
          id: "2",
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
            // Must match the mapping in ros_subscriber.py and MoveIt2 service names
            const taskKeyToId: { [key: string]: string } = {
              set_up_table: "1",      // Task 1 = Set up the table
              prepare_medicine: "2",  // Task 2 = Prepare medicine
              organize_books: "3",    // Task 3 = Organize books
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
    if (!connected || !ros) return;

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
          console.log("[TaskContext] Received ROS progress update:", progress);
          setCurrentProgress(progress);

          // Update task state based on progress
          setTasks((prevTasks) =>
            prevTasks.map((task) => {
              const taskKey = TASK_SERVICE_MAP[task.id];
              if (taskKey === progress.task) {
                console.log(`[TaskContext] Updating task ${task.id} with progress: step=${progress.current_step}, status=${progress.status}, progress=${progress.progress_percent}%`);
                
                // If task failed, reset UI to initial state (progress 0%, step 1 highlighted)
                if (progress.status === "failed") {
                  console.log(`[TaskContext] Task ${task.id} failed - resetting UI to initial state`);
                  const resetSteps = task.steps.map((step, index) => ({
                    ...step,
                    status: index === 0 ? "in-progress" as const : "not-started" as const,
                  }));
                  
                  const updatedTask = {
                    ...task,
                    progress: 0,
                    currentStep: task.steps[0]?.title || "Ready to start",
                    status: "failed" as const,
                    steps: resetSteps,
                  };
                  
                  console.log(`[TaskContext] Reset task ${task.id}: progress=0%, step 1 highlighted, all other steps not-started`);
                  return updatedTask;
                }
                
                // Update steps status for normal progress
                // When current_step = 2 and status = "in-progress", it means:
                // - Step 1 is completed (stepNumber < current_step)
                // - Step 2 is in-progress (stepNumber === current_step)
                // When current_step = 1 and status = "completed", it means:
                // - Step 1 is completed
                const updatedSteps = task.steps.map((step, index) => {
                  const stepNumber = index + 1;
                  let status:
                    | "not-started"
                    | "in-progress"
                    | "completed"
                    | "failed" = "not-started";

                  if (stepNumber < progress.current_step) {
                    // Previous steps are completed (remain highlighted)
                    // When current_step = 2, step 1 (stepNumber=1) is completed
                    // When current_step = 3, steps 1-2 are completed, etc.
                    status = "completed";
                  } else if (stepNumber === progress.current_step) {
                    // Current step status matches progress status
                    if (progress.status === "completed") {
                      // If current step is marked as completed, mark it as completed
                      status = "completed";
                    } else {
                      // Otherwise it's in-progress
                      status = "in-progress";
                    }
                  } else {
                    // Future steps (stepNumber > current_step) remain "not-started"
                    status = "not-started";
                  }

                  return { ...step, status };
                });

                // Calculate progress based on COMPLETED steps only
                // Progress = (completed_steps / total_steps) * 100
                // Only count steps that are actually completed, not in-progress
                // This ensures progress bar only updates when a step is fully executed
                const completedSteps = updatedSteps.filter(s => s.status === "completed").length;
                const calculatedProgress = Math.round((completedSteps / task.totalSteps) * 100);
                
                console.log(`[TaskContext] Progress calculation: completedSteps=${completedSteps}, totalSteps=${task.totalSteps}, progress=${calculatedProgress}%`);
                console.log(`[TaskContext] Step statuses:`, updatedSteps.map((s, i) => `Step ${i + 1}: ${s.status}`));
                
                const updatedTask = {
                  ...task,
                  progress: calculatedProgress,
                  currentStep: progress.description,
                  status: progress.status,
                  steps: updatedSteps,
                };
                
                console.log(`[TaskContext] Updated task ${task.id} steps:`, updatedSteps.map(s => `${s.title}: ${s.status}`));
                
                return updatedTask;
              }
              return task;
            }),
          );
        } catch (e) {
          console.error("[TaskContext] Failed to parse progress update:", e);
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
      if (!serviceName) {
        console.error(`Invalid task ID: ${taskId}`);
        return;
      }
      if (!ros) {
        console.error("Cannot start task: ROS object not available");
        return;
      }
      
      // Log connection status but don't block - let the service call fail if not connected
      if (!connected) {
        console.warn(`[Task] ROS connection status: connected=${connected}, but attempting service call anyway`);
      } else {
        console.log(`[Task] ROS connection status: connected=${connected}, ros=${!!ros}`);
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

  // Stop a running task
  const stopTask = useCallback(
    (taskId: string) => {
      console.log(`[Task] Stopping task ${taskId}`);
      
      // Clear paused state when stopping
      setPausedTasks((prev) => {
        const newSet = new Set(prev);
        newSet.delete(taskId);
        return newSet;
      });
      
      // Update task status to not-started
      setTasks((prevTasks) =>
        prevTasks.map((task) =>
          task.id === taskId
            ? { ...task, status: "not-started" as const, progress: 0 }
            : task,
        ),
      );

      // Publish stop command to ROS to stop the robotic arm immediately
      if (connected && ros) {
        try {
          // eslint-disable-next-line @typescript-eslint/no-var-requires
          const ROSLIB = require("roslib");
          const stopTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/robot/stop_task",
            messageType: "std_msgs/String",
          });

          const stopMessage = new ROSLIB.Message({
            data: JSON.stringify({ task_id: taskId, command: "stop" }),
          });

          stopTopic.publish(stopMessage);
          console.log(`[Task] Published stop command to /robot/stop_task for task ${taskId}`);
        } catch (e) {
          console.error(`[Task] Error publishing stop command:`, e);
        }
      } else {
        console.warn(`[Task] Cannot stop task: ROS not connected (connected=${connected}, ros=${!!ros})`);
      }
    },
    [connected, ros],
  );

  // Pause a running task
  const pauseTask = useCallback(
    (taskId: string) => {
      console.log(`[Task] Pausing task ${taskId}`);
      
      // Mark task as paused
      setPausedTasks((prev) => new Set(prev).add(taskId));

      // Publish pause command to ROS to pause the robotic arm
      if (connected && ros) {
        try {
          // eslint-disable-next-line @typescript-eslint/no-var-requires
          const ROSLIB = require("roslib");
          const pauseTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/robot/pause_task",
            messageType: "std_msgs/String",
          });

          const pauseMessage = new ROSLIB.Message({
            data: JSON.stringify({ task_id: taskId, command: "pause" }),
          });

          pauseTopic.publish(pauseMessage);
          console.log(`[Task] Published pause command to /robot/pause_task for task ${taskId}`);
        } catch (e) {
          console.error(`[Task] Error publishing pause command:`, e);
        }
      } else {
        console.warn(`[Task] Cannot pause task: ROS not connected (connected=${connected}, ros=${!!ros})`);
      }
    },
    [connected, ros],
  );

  // Resume a paused task
  const resumeTask = useCallback(
    (taskId: string) => {
      console.log(`[Task] Resuming task ${taskId}`);
      
      // Remove task from paused set
      setPausedTasks((prev) => {
        const newSet = new Set(prev);
        newSet.delete(taskId);
        return newSet;
      });

      // Publish resume command to ROS to resume the robotic arm
      if (connected && ros) {
        try {
          // eslint-disable-next-line @typescript-eslint/no-var-requires
          const ROSLIB = require("roslib");
          const resumeTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/robot/resume_task",
            messageType: "std_msgs/String",
          });

          const resumeMessage = new ROSLIB.Message({
            data: JSON.stringify({ task_id: taskId, command: "resume" }),
          });

          resumeTopic.publish(resumeMessage);
          console.log(`[Task] Published resume command to /robot/resume_task for task ${taskId}`);
        } catch (e) {
          console.error(`[Task] Error publishing resume command:`, e);
        }
      } else {
        console.warn(`[Task] Cannot resume task: ROS not connected (connected=${connected}, ros=${!!ros})`);
      }
    },
    [connected, ros],
  );

  // Check if a task is paused
  const isPaused = useCallback(
    (taskId: string) => {
      return pausedTasks.has(taskId);
    },
    [pausedTasks],
  );

  // Clear paused state when task stops
  useEffect(() => {
    tasks.forEach((task) => {
      if (task.status === "not-started" || task.status === "completed" || task.status === "failed") {
        setPausedTasks((prev) => {
          const newSet = new Set(prev);
          newSet.delete(task.id);
          return newSet;
        });
      }
    });
  }, [tasks]);

  return (
    <TaskContext.Provider
      value={{
        selectedId,
        setSelectedId,
        tasks,
        currentProgress,
        loadTasks,
        startTask,
        stopTask,
        pauseTask,
        resumeTask,
        isPaused,
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
