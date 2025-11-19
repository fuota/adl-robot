import React, { createContext, PropsWithChildren, useContext, useEffect, useState } from 'react';
import { Platform } from 'react-native';

interface RosContextProps {
  ros: any;
  connected: boolean;
}

const RosContext = createContext<RosContextProps | null>(null);

export const RosProvider = ({ children }: PropsWithChildren) => {
  const isNative = Platform.OS !== 'web';

  // Minimal synchronous stub so the provider can render immediately on native
  // platforms (Expo Go on iPad) without attempting any network connections.
  const nativeStub = {
    on: (_: string, __?: any) => {},
    close: () => {},
  };

  const [ros, setRos] = useState<any>(nativeStub);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    // Only initialize the real ROS websocket on web builds. Native builds
    // keep the synchronous stub so UI renders immediately in Expo Go.
    if (isNative) return;

    // Dynamically require roslib so native bundlers don't include it.
    // eslint-disable-next-line @typescript-eslint/no-var-requires
    const ROSLIB = require('roslib');

    const rosConnection = new ROSLIB.Ros({ url: 'ws://192.168.0.217:9090' });

    rosConnection.on('connection', () => {
      console.log('Connected to ROS bridge');
      setConnected(true);
    });

    rosConnection.on('error', (err: any) => {
      console.error('ROS bridge error:', err);
      setConnected(false);
    });

    rosConnection.on('close', () => {
      console.log('ROS bridge closed');
      setConnected(false);
    });

    setRos(rosConnection);

    return () => {
      try {
        rosConnection.close();
      } catch (_e) {
        // ignore
      }
    };
  }, [isNative]);

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
