import React, { createContext, PropsWithChildren, useContext, useEffect, useState } from 'react';

interface RosContextProps {
  ros: any;
  connected: boolean;
}

const RosContext = createContext<RosContextProps | null>(null);

export const RosProvider = ({ children }: PropsWithChildren) => {
  // Minimal synchronous stub for initial render
  const initialStub = {
    on: (_: string, __?: any) => {},
    close: () => {},
  };

  const [ros, setRos] = useState<any>(initialStub);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    // Dynamically require roslib so native bundlers don't include it.
    // eslint-disable-next-line @typescript-eslint/no-var-requires
    const ROSLIB = require('roslib');

    console.log('[ROS] Initializing ROS connection to ws://192.168.0.217:9090');
    const rosConnection = new ROSLIB.Ros({ url: 'ws://192.168.0.217:9090' });

    // Check connection state periodically
    const checkConnection = () => {
      try {
        // ROSLIB connection has an isConnected property
        const isConnected = (rosConnection as any).isConnected === true;
        if (isConnected) {
          setConnected((prev) => {
            if (!prev) {
              console.log('[ROS] Connection detected via isConnected check');
            }
            return true;
          });
        }
      } catch (e) {
        // ignore
      }
    };

    rosConnection.on('connection', () => {
      console.log('[ROS] Connected to ROS bridge');
      setConnected(true);
    });

    rosConnection.on('error', (err: any) => {
      console.error('[ROS] ROS bridge error:', err);
      setConnected(false);
    });

    rosConnection.on('close', () => {
      console.log('[ROS] ROS bridge closed');
      setConnected(false);
    });

    // Set ROS connection immediately (don't wait for connection event)
    setRos(rosConnection);
    console.log('[ROS] ROS connection object created');

    // Check connection state periodically (every 2 seconds)
    const connectionCheckInterval = setInterval(checkConnection, 2000);

    // Also check immediately
    setTimeout(checkConnection, 1000);

    return () => {
      clearInterval(connectionCheckInterval);
      try {
        rosConnection.close();
      } catch (_e) {
        // ignore
      }
    };
  }, []);

  return (
    <RosContext.Provider value={{ ros, connected }}>
      {children}
    </RosContext.Provider>
  );
};

export const useRos = (): RosContextProps => {
  const context = useContext(RosContext);
  if (!context) throw new Error('useRos must be used within a RosProvider');
  return context;
};
