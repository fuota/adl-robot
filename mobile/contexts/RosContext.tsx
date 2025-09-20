import React, { createContext, PropsWithChildren, useContext, useEffect, useState } from 'react';
import ROSLIB from 'roslib';

interface RosContextProps {
  ros: ROSLIB.Ros;
  connected: boolean;
}

const RosContext = createContext<RosContextProps | null>(null);

export const RosProvider = ({ children }: PropsWithChildren) => {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const rosConnection = new ROSLIB.Ros({
      url: 'ws://192.168.0.217:9090'
    });

    rosConnection.on('connection', () => {
      console.log('Connected to ROS bridge');
      setConnected(true);
    });

    rosConnection.on('error', (err) => {
      console.error('ROS bridge error:', err);
      setConnected(false);
    });

    rosConnection.on('close', () => {
      console.log('ROS bridge closed');
      setConnected(false);
    });

    setRos(rosConnection);

    return () => {
      rosConnection.close();
    };
  }, []);

  // Only render children when ROS is initialized
  if (!ros) return null;

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
