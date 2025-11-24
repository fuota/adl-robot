import { useEffect, useRef } from 'react';
import { Platform, DeviceEventEmitter } from 'react-native';
import { BACKEND_URL } from '@/config/backend';

/**
 * WebSocket hook for real-time task updates
 * Connects to backend WebSocket server and emits taskStepUpdate events
 */
export function useTaskWebSocket(taskId: string | undefined) {
  const socketRef = useRef<any>(null);
  const reconnectTimeoutRef = useRef<any>(null);
  const reconnectAttempts = useRef(0);
  const maxReconnectAttempts = 5;

  useEffect(() => {
    if (!taskId) return;

    // Try to use Socket.IO client (works on web and can work on native with proper setup)
    let SocketIO: any = null;
    try {
      // eslint-disable-next-line @typescript-eslint/no-var-requires
      SocketIO = require('socket.io-client');
    } catch (e) {
      console.log('[WebSocket] Socket.IO client not available, using polling fallback');
    }

    if (SocketIO && Platform.OS === 'web') {
      // Use Socket.IO client for web platform
      const connectSocket = () => {
        try {
          const socket = SocketIO(BACKEND_URL, {
            transports: ['websocket', 'polling'],
            reconnection: true,
            reconnectionDelay: 1000,
            reconnectionAttempts: maxReconnectAttempts,
          });

          socket.on('connect', () => {
            console.log('[WebSocket] Connected to task update server');
            reconnectAttempts.current = 0;
          });

          socket.on('connected', (data: any) => {
            console.log('[WebSocket] Server confirmed connection:', data);
          });

          socket.on('task_step_update', (update: {
            taskId?: string;
            stepIndex?: number;
            action?: string;
            status?: string;
          }) => {
            // Only process updates for current task
            if (!update.taskId || String(update.taskId) === String(taskId)) {
              console.log('[WebSocket] Processing task step update:', update);
              
              // Emit event for DeviceEventEmitter (native) or window event (web)
              if (Platform.OS !== 'web') {
                DeviceEventEmitter.emit('taskStepUpdate', {
                  taskId: update.taskId || taskId,
                  stepIndex: update.stepIndex,
                  action: update.action,
                  status: update.status || 'completed',
                });
              } else {
                // For web, dispatch custom event
                const event = new CustomEvent('taskStepUpdate', {
                  detail: {
                    taskId: update.taskId || taskId,
                    stepIndex: update.stepIndex,
                    action: update.action,
                    status: update.status || 'completed',
                  },
                });
                window.dispatchEvent(event);
              }
            }
          });

          socket.on('disconnect', () => {
            console.log('[WebSocket] Disconnected from server');
          });

          socket.on('connect_error', (error: any) => {
            console.error('[WebSocket] Connection error:', error);
          });

          socketRef.current = socket;
        } catch (error) {
          console.error('[WebSocket] Failed to connect:', error);
        }
      };

      connectSocket();

      return () => {
        if (socketRef.current) {
          socketRef.current.disconnect();
          socketRef.current = null;
        }
        if (reconnectTimeoutRef.current) {
          clearTimeout(reconnectTimeoutRef.current);
        }
      };
    } else {
      // For native platforms or when Socket.IO is not available, use polling
      console.log('[Task Updates] Using polling fallback');
      let previousProgress = -1;
      
      const pollInterval = setInterval(async () => {
        try {
          const response = await fetch(`${BACKEND_URL}/task/status`);
          const state = await response.json();
          
          // Check if task state has updated
          if (state.task_id && String(state.task_id) === String(taskId)) {
            // Check if progress changed
            if (state.progress !== previousProgress && state.actions) {
              previousProgress = state.progress;
              
              // Find completed actions and emit updates
              state.actions.forEach((action: any, index: number) => {
                if (action.status === 'done' || action.status === 'completed') {
                  // Emit update for this step
                  if (Platform.OS !== 'web') {
                    DeviceEventEmitter.emit('taskStepUpdate', {
                      taskId: String(taskId),
                      stepIndex: index,
                      action: action.name,
                      status: 'completed',
                    });
                  } else {
                    const event = new CustomEvent('taskStepUpdate', {
                      detail: {
                        taskId: String(taskId),
                        stepIndex: index,
                        action: action.name,
                        status: 'completed',
                      },
                    });
                    window.dispatchEvent(event);
                  }
                }
              });
            }
          }
        } catch (error) {
          console.error('[Polling] Error fetching task status:', error);
        }
      }, 2000); // Poll every 2 seconds

      return () => {
        clearInterval(pollInterval);
      };
    }
  }, [taskId]);
}

