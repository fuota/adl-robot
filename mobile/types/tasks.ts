/**
 * Task Types - Synced with ROS backend
 */

export type TaskStatus = "not-started" | "in_progress" | "completed" | "failed";

export interface TaskStep {
  step: number;
  description: string;
}

export interface TaskDefinition {
  name: string;
  description: string;
  total_steps: number;
  steps: TaskStep[];
  estimated_duration_seconds: number;
}

export interface TaskProgress {
  task: string;
  current_step: number;
  total_steps: number;
  description: string;
  status: TaskStatus;
  progress_percent: number;
}

export interface AvailableTasksResponse {
  tasks: {
    [key: string]: TaskDefinition;
  };
  timestamp: number;
}

// UI representation of a task
export interface Task {
  id: string;
  title: string;
  description: string;
  progress: number;
  currentStep: string;
  currentStepNumber: number;
  status: TaskStatus;
  steps: Array<{
    title: string;
    status: TaskStatus;
  }>;
  totalSteps: number;
  estimatedDuration: number;
}
