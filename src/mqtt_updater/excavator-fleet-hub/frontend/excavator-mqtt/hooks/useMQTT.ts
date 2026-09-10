 "use client";

import { useCallback, useEffect, useRef, useState } from "react";
import mqtt, { type MqttClient } from "mqtt";

interface UseMQTTResult {
  client: MqttClient | null;
  isConnected: boolean;
  lastMessage: string | null;
  error: string | null;
  publish: (topic: string, payload: string, options?: any) => void;
}

export function useMQTT(
  brokerUrl: string,
  topic?: string | string[]
): UseMQTTResult {
  const clientRef = useRef<MqttClient | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [lastMessage, setLastMessage] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (typeof window === "undefined") return;
    if (!brokerUrl) return;

    const client = mqtt.connect(brokerUrl, {
      clientId: `web-fleet-${Math.random().toString(16).slice(2)}`,
      clean: true,
      reconnectPeriod: 3000,
    });

    clientRef.current = client;

    client.on("connect", () => {
      setIsConnected(true);
      setError(null);

      if (topic) {
        const topics = Array.isArray(topic) ? topic : [topic];
        topics.forEach((t) => client.subscribe(t));
      }
    });

    client.on("reconnect", () => {
      setIsConnected(false);
    });

    client.on("error", (err) => {
      setError(err.message);
    });

    client.on("message", (_topic, payload) => {
      setLastMessage(payload.toString());
    });

    return () => {
      client.end(true);
      clientRef.current = null;
    };
  }, [brokerUrl, topic]);

  const publish = useCallback(
    (targetTopic: string, payload: string, options?: any) => {
      if (!clientRef.current) return;
      clientRef.current.publish(targetTopic, payload, options);
    },
    []
  );

  return {
    client: clientRef.current,
    isConnected,
    lastMessage,
    error,
    publish,
  };
}

