import os, time, paho.mqtt.client as mqtt
c=mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="cc-sub")
c.username_pw_set("innocowuser", os.environ["MQTT_PASS"]); c.tls_set()
c.on_message=lambda cl,u,m: print("📥 SUB", m.topic, m.payload.decode()[:90], flush=True)
c.connect("9d3a3c61e89e4384ab8e326039d9f567.s1.eu.hivemq.cloud",8883); c.subscribe("projekt/#"); c.loop_start(); time.sleep(float(os.environ.get("SUB_SECS","12")))
