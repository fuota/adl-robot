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
  callPrimitiveAction: (action: string, objectId?: string, taskId?: string) => void;
  detectedObjects: { [key: string]: { name: string; has_pose: boolean } };
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
  const [detectedObjects, setDetectedObjects] = useState<{ [key: string]: { name: string; has_pose: boolean } }>({});
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
      currentStepNumber: 1,
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
          currentStepNumber: 1,
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
          currentStepNumber: 1,
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
          currentStepNumber: 1,
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
          console.log("[TaskContext] Received ROS progress update:", JSON.stringify(progress, null, 2));
          setCurrentProgress(progress);

          // Normalize task names for comparison (case-insensitive, handle underscores/hyphens/spaces)
          const normalizeTaskName = (name: string): string => {
            return name.toLowerCase().replace(/[-_\s]/g, "").trim();
          };

          // Update task state based on progress
          setTasks((prevTasks) => {
            // First, log all progress updates for debugging
            console.log(`[TaskContext] Processing progress update for task: "${progress.task}"`);
            console.log(`[TaskContext] Available tasks:`, prevTasks.map(t => ({ id: t.id, title: t.title, status: t.status })));
            
            return prevTasks.map((task) => {
              const taskKey = TASK_SERVICE_MAP[task.id];
              const normalizedTaskKey = normalizeTaskName(taskKey || "");
              const normalizedProgressTask = normalizeTaskName(progress.task || "");
              const isMatch = normalizedTaskKey === normalizedProgressTask;
              
              // Enhanced logging for Task 2 specifically
              if (task.id === "2") {
                console.log(`[TaskContext] Task 2 matching check:`);
                console.log(`  - taskKey (raw): "${taskKey}"`);
                console.log(`  - progress.task (raw): "${progress.task}"`);
                console.log(`  - normalizedTaskKey: "${normalizedTaskKey}"`);
                console.log(`  - normalizedProgressTask: "${normalizedProgressTask}"`);
                console.log(`  - Match: ${isMatch}`);
                console.log(`  - Current step: ${progress.current_step}/${progress.total_steps}`);
                console.log(`  - Status: ${progress.status}`);
                console.log(`  - Description: "${progress.description}"`);
                console.log(`  - Progress percent: ${progress.progress_percent}`);
              } else {
                console.log(`[TaskContext] Checking task ${task.id}: taskKey="${taskKey}", progress.task="${progress.task}", match=${isMatch}`);
              }
              
              if (isMatch) {
                if (progress.status === "failed") {
                  console.log(`[TaskContext] Task ${task.id} failed - resetting UI state`);
                  const resetSteps = task.steps.map((step, index) => ({
                    ...step,
                    status: index === 0 ? ("in-progress" as const) : ("not-started" as const),
                  }));

                  return {
                    ...task,
                    progress: 0,
                    currentStep: task.steps[0]?.title || "Ready to start",
                    currentStepNumber: 1,
                    status: "failed" as const,
                    steps: resetSteps,
                  };
                }

                // Update steps status with ROS data
                // ROS publishes status for the current step only (in_progress, completed, failed)
                const currentStepNum = progress.current_step || 1;
                const updatedSteps = task.steps.map((step, index) => {
                  const stepNumber = index + 1;
                  let title = step.title;
                  let status:
                    | "not-started"
                    | "in-progress"
                    | "completed"
                    | "failed" = step.status; // Preserve existing status by default

                  // If this is the current step, update its status from ROS
                  if (stepNumber === currentStepNum) {
                    // ROS status applies to this current step
                    // Normalize ROS status: "in_progress" -> "in-progress" (ROS may use underscore)
                    let rosStatus: string = progress.status || "";
                    if (rosStatus === "in_progress" || rosStatus === "in-progress") {
                      status = "in-progress";
                    } else if (rosStatus === "completed") {
                      status = "completed";
                    } else if (rosStatus === "failed") {
                      status = "failed";
                    }
                    
                    if (status !== step.status) {
                      if (task.id === "2") {
                        console.log(`[TaskContext] Task 2 - Step ${stepNumber} (${title}): ${status} (from ROS, was: ${step.status})`);
                      } else {
                        console.log(`[TaskContext] Step ${stepNumber} (${title}): ${status} (from ROS)`);
                      }
                    }

                    // Update title from ROS description if available
                    if (progress.description) {
                      title = progress.description;
                    }
                  }
                  // If this step is before the current step, mark as completed (unless already failed)
                  else if (stepNumber < currentStepNum) {
                    // Only mark as completed if not already failed
                    if (step.status !== "failed") {
                      status = "completed";
                      if (task.id === "2") {
                        console.log(`[TaskContext] Task 2 - Step ${stepNumber} (${title}): completed (previous step)`);
                      }
                    }
                  }
                  // Future steps remain not-started (or keep their existing status if already set)

                  return { ...step, status, title };
                });

                // Use progress_percent directly from ROS if available, otherwise calculate from completed steps
                // ROS provides progress_percent in the /task_progress message
                const rosProgressPercent = progress.progress_percent;
                const totalStepsForProgress = progress.total_steps || task.totalSteps || updatedSteps.length;
                
                let finalProgress: number;
                if (rosProgressPercent !== undefined && rosProgressPercent !== null) {
                  // Use ROS-provided progress_percent directly
                  finalProgress = Math.round(rosProgressPercent);
                  console.log(`[TaskContext] Using ROS progress_percent: ${finalProgress}%`);
                } else {
                  // Fallback: Calculate progress based on completed steps
                  const completedSteps = updatedSteps.filter(s => s.status === "completed").length;
                  finalProgress = Math.round((completedSteps / totalStepsForProgress) * 100);
                  console.log(`[TaskContext] ROS progress_percent not available, calculating from completed steps: ${finalProgress}%`);
                }
                
                // Enhanced logging for all tasks
                if (task.id === "2") {
                  console.log(`[TaskContext] Task 2 Progress:`);
                  console.log(`  - ROS progress_percent: ${rosProgressPercent !== undefined ? rosProgressPercent : 'not provided'}`);
                  console.log(`  - Final progress: ${finalProgress}%`);
                  console.log(`  - Current step: ${progress.current_step}/${totalStepsForProgress}`);
                  console.log(`  - Status: ${progress.status}`);
                  console.log(`  - Step statuses:`, updatedSteps.map((s, i) => `Step ${i + 1} (${s.title}): ${s.status}`));
                } else {
                  console.log(`[TaskContext] Task ${task.id} Progress: ROS=${rosProgressPercent !== undefined ? rosProgressPercent : 'N/A'}, Final=${finalProgress}%`);
                  console.log(`[TaskContext] Step statuses:`, updatedSteps.map((s, i) => `Step ${i + 1}: ${s.status}`));
                }
                
                // Determine overall task status based on step statuses:
                // - "failed" if any step failed
                // - "completed" if ALL steps are completed
                // - "running" (in-progress) if task started OR any step is in-progress (but not all completed)
                // - "idle" (not-started) if all steps are not-started
                let finalStatus: "not-started" | "in-progress" | "completed" | "failed";
                
                // Check if any step failed
                const hasFailedStep = updatedSteps.some(s => s.status === "failed");
                if (hasFailedStep) {
                  finalStatus = "failed";
                  if (task.id === "2") {
                    console.log(`[TaskContext] Task 2: Overall status = "failed" (step failed)`);
                  }
                }
                // Check if ALL steps are completed
                else if (updatedSteps.length > 0 && updatedSteps.every(s => s.status === "completed")) {
                  finalStatus = "completed";
                  if (task.id === "2") {
                    console.log(`[TaskContext] Task 2: Overall status = "completed" (all steps finished)`);
                  }
                }
                // Check if task has been started (any step is in-progress or completed, but not all completed)
                else if (updatedSteps.some(s => s.status === "in-progress" || s.status === "completed") || task.status === "in-progress") {
                  finalStatus = "in-progress";
                  if (task.id === "2") {
                    console.log(`[TaskContext] Task 2: Overall status = "running" (task active)`);
                  }
                }
                // All steps are not-started
                else {
                  finalStatus = "not-started";
                  if (task.id === "2") {
                    console.log(`[TaskContext] Task 2: Overall status = "idle" (not started)`);
                  }
                }
                
                const updatedTask = {
                  ...task,
                  progress: finalProgress,
                  currentStep: progress.description || task.currentStep,
                  currentStepNumber: progress.current_step || task.currentStepNumber || 1,
                  totalSteps: totalStepsForProgress,
                  status: finalStatus,
                  steps: updatedSteps,
                };
                
                if (task.id === "2") {
                  console.log(`[TaskContext] Task 2 Updated - Final state:`);
                  console.log(`  - Progress: ${updatedTask.progress}%`);
                  console.log(`  - Current Step: ${updatedTask.currentStepNumber}/${updatedTask.totalSteps}`);
                  console.log(`  - Status: ${updatedTask.status}`);
                  console.log(`  - Steps:`, updatedTask.steps.map(s => `${s.title}: ${s.status}`));
                } else {
                  console.log(`[TaskContext] Updated task ${task.id} steps:`, updatedSteps.map(s => `${s.title}: ${s.status}`));
                }
                
                return updatedTask;
              }
              return task;
            });
          });
        } catch (e) {
          console.error("[TaskContext] Failed to parse progress update:", e);
          console.error("[TaskContext] Raw message data:", message?.data);
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

  // Subscribe to detected objects
  useEffect(() => {
    if (!connected || !ros) return;

    try {
      // eslint-disable-next-line @typescript-eslint/no-var-requires
      const ROSLIB = require("roslib");
      const detectedObjectsTopic = new ROSLIB.Topic({
        ros: ros,
        name: "/robot/detected_objects",
        messageType: "std_msgs/String",
      });

      detectedObjectsTopic.subscribe((message: any) => {
        try {
          const objects = JSON.parse(message.data);
          setDetectedObjects(objects);
          console.log("[TaskContext] Received detected objects:", objects);
        } catch (e) {
          console.error("[TaskContext] Failed to parse detected objects:", e);
        }
      });

      return () => {
        try {
          detectedObjectsTopic.unsubscribe();
        } catch (e) {
          console.error("Error unsubscribing from detected_objects:", e);
        }
      };
    } catch (e) {
      console.error("Error subscribing to detected_objects:", e);
    }
  }, [connected, ros]);

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
              ? { 
                  ...task, 
                  status: "in-progress" as const, 
                  progress: 0,
                  currentStepNumber: 1,
                  currentStep: task.steps[0]?.title || "Starting..."
                }
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
      
      // Update task status to not-started
      setTasks((prevTasks) =>
        prevTasks.map((task) =>
          task.id === taskId
            ? { 
                ...task, 
                status: "not-started" as const, 
                progress: 0,
                currentStepNumber: 1,
                currentStep: task.steps[0]?.title || "Ready"
              }
            : task,
        ),
      );

      // Call ROS service to stop the robotic arm immediately
      if (connected && ros) {
        try {
          // eslint-disable-next-line @typescript-eslint/no-var-requires
          const ROSLIB = require("roslib");
          
          // Call emergency_stop ROS service
          const emergencyStopService = new ROSLIB.Service({
            ros: ros,
            name: "/emergency_stop",
            serviceType: "std_srvs/Trigger",
          });

          const request = new ROSLIB.ServiceRequest({});

          emergencyStopService.callService(
            request,
            (result: any) => {
              console.log(`[Task] Emergency stop service call successful for task ${taskId}:`, result.success ? "Success" : "Failed");
            },
            (error: any) => {
              // If service call fails, fall back to publishing to topic
              console.log(`[Task] Emergency stop service /emergency_stop not available, falling back to topic publish:`, error);
              try {
                const stopTopic = new ROSLIB.Topic({
                  ros: ros,
                  name: "/robot/stop_task",
                  messageType: "std_msgs/String",
                });

                const stopMessage = new ROSLIB.Message({
                  data: JSON.stringify({ task_id: taskId, command: "stop" }),
                });

                stopTopic.publish(stopMessage);
                console.log(`[Task] Published stop command to /robot/stop_task topic for task ${taskId}`);
              } catch (topicPublishError) {
                console.error(`[Task] Error publishing stop command to topic:`, topicPublishError);
              }
            }
          );
        } catch (e) {
          console.error(`[Task] Error calling emergency stop service:`, e);
        }
      } else {
        console.warn(`[Task] Cannot stop task: ROS not connected (connected=${connected}, ros=${!!ros})`);
      }
    },
    [connected, ros],
  );

  // Call a primitive action (e.g., "grasp", "lift", "drop") with object ID
  const callPrimitiveAction = useCallback(
    (action: string, objectId?: string, taskId?: string) => {
      console.log(`[Task] Calling primitive action: ${action}${objectId ? ` for object ${objectId}` : ""}${taskId ? ` in task ${taskId}` : ""}`);
      
      if (!connected || !ros) {
        console.warn(`[Task] Cannot call primitive action: ROS not connected (connected=${connected}, ros=${!!ros})`);
        return;
      }

      if (!objectId) {
        console.error(`[Task] Cannot call primitive action: objectId is required`);
        return;
      }

      try {
        // eslint-disable-next-line @typescript-eslint/no-var-requires
        const ROSLIB = require("roslib");
        
        // Try calling as a ROS service first (e.g., /grasp, /lift, /drop)
        // Service should accept object_id as parameter
        const actionService = new ROSLIB.Service({
          ros: ros,
          name: `/${action}`,
          serviceType: "std_srvs/SetBool", // Or appropriate service type that accepts object_id
        });

        // Try with SetBool service type first
        const request = new ROSLIB.ServiceRequest({
          data: objectId, // Pass object_id as data
        });

        actionService.callService(
          request,
          (result: any) => {
            console.log(`[Task] Primitive action ${action} for object ${objectId} completed:`, result.success ? "Success" : "Failed");
          },
          (error: any) => {
            // If SetBool fails, try with String service type
            console.log(`[Task] Service call with SetBool failed, trying String service:`, error);
            
            try {
              const stringService = new ROSLIB.Service({
                ros: ros,
                name: `/${action}`,
                serviceType: "std_srvs/Trigger",
              });

              const triggerRequest = new ROSLIB.ServiceRequest({});

              stringService.callService(
                triggerRequest,
                (result: any) => {
                  console.log(`[Task] Primitive action ${action} (trigger) completed:`, result.success ? "Success" : "Failed");
                },
                (triggerError: any) => {
                  // If service calls fail, publish to topic with object_id
                  console.log(`[Task] Service calls failed, trying topic publish for ${action}:`, triggerError);
                  
                  try {
                    const actionTopic = new ROSLIB.Topic({
                      ros: ros,
                      name: `/robot/primitive_action`,
                      messageType: "std_msgs/String",
                    });

                    const actionMessage = new ROSLIB.Message({
                      data: JSON.stringify({ 
                        action: action,
                        object_id: objectId,
                        task_id: taskId || "" 
                      }),
                    });

                    actionTopic.publish(actionMessage);
                    console.log(`[Task] Published primitive action ${action} for object ${objectId} to /robot/primitive_action`);
                  } catch (topicError) {
                    console.error(`[Task] Error publishing primitive action:`, topicError);
                  }
                }
              );
            } catch (stringServiceError) {
              console.error(`[Task] Error calling string service:`, stringServiceError);
              // Fallback to topic
              try {
                const actionTopic = new ROSLIB.Topic({
                  ros: ros,
                  name: `/robot/primitive_action`,
                  messageType: "std_msgs/String",
                });

                const actionMessage = new ROSLIB.Message({
                  data: JSON.stringify({ 
                    action: action,
                    object_id: objectId,
                    task_id: taskId || "" 
                  }),
                });

                actionTopic.publish(actionMessage);
                console.log(`[Task] Published primitive action ${action} for object ${objectId} to /robot/primitive_action`);
              } catch (topicError) {
                console.error(`[Task] Error publishing primitive action:`, topicError);
              }
            }
          },
        );
      } catch (e) {
        console.error(`[Task] Error calling primitive action ${action}:`, e);
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
        stopTask,
        callPrimitiveAction,
        detectedObjects,
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
