import React, { createContext, useState, useContext } from "react";

type TaskContextType = {
  selectedId: string | null;
  setSelectedId: (id: string | null) => void;
};

const TaskContext = createContext<TaskContextType | undefined>(undefined);

export const TaskProvider = ({ children }: { children: React.ReactNode }) => {
  const [selectedId, setSelectedId] = useState<string | null>(null);
  return <TaskContext.Provider value={{ selectedId, setSelectedId }}>{children}</TaskContext.Provider>;
};

export const useTask = () => {
  const ctx = useContext(TaskContext);
  if (!ctx) throw new Error("useTask must be used within TaskProvider");
  return ctx;
};

export default TaskContext;
