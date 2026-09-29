#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import subprocess
import time
import sys

MQTT_HOST = "192.168.0.174"
MQTT_PORT = 1883
SOUNDS_DIR = "/home/quarl/sounds"
start_time = time.time()

# One-time stage alarms
TOPIC_SOUNDS = {
    "mashTun/mash_instep_alarm": "0_maskningsVattentArUppeITemperatur.wav",
    "mashTun/lautering_alarm": "1_maskningenFardig_v2.wav",
    "mashTun/boil_alarm": "2_lakningenFardig_v2.wav",
    "mashTun/hops_alarm": "3_dagsAttLaggaIHumlen_v2.wav",
    "mashTun/whirlpool_alarm": "4_kokning_fardig_v2.wav",
    "mashTun/whirlpool_finished_alarm": "5_virvlingenArFarig.wav",
}
# Repeating "still not switched" reminders (every 5 min from firmware)
REMINDER_SOUNDS = {
    "mashTun/mash_instep_reminder": "0.1_maskningsVattentArUppeITemperatur_ForHel.wav",
    "mashTun/lautering_reminder": "1.1_maskningenArFardigForHel.wav",
    "mashTun/boil_reminder": "2.1_lakningenArFardigForHel.wav",
    "mashTun/whirlpool_reminder": "4.1_kokningenArFardigForHel.wav",
    "mashTun/whirlpool_finished_reminder": "5.1_virvlingenArFarig_forHel.wav",
}
ALL_TOPICS = list(TOPIC_SOUNDS.keys()) + list(REMINDER_SOUNDS.keys())


def play_sound(sound_file, retries=3, delay=2):
    """Play a sound via paplay, retrying if the audio server isn't ready yet."""
    for attempt in range(1, retries + 1):
        result = subprocess.run(
            ["paplay", sound_file],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        if result.returncode == 0:
            return
        print(
            f"  -> paplay failed (attempt {attempt}/{retries}) for {sound_file}: "
            f"{result.stderr.strip()}",
            file=sys.stderr,
        )
        time.sleep(delay)
    print(f"  -> Giving up on playing {sound_file} after {retries} attempts", file=sys.stderr)


def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Connected to MQTT broker")
    else:
        print(f"Connect failed with rc={rc}", file=sys.stderr)
    for topic in ALL_TOPICS:
        client.subscribe(topic)
        print(f"Subscribed to {topic}")


def on_disconnect(client, userdata, rc, properties=None):
    print(f"Disconnected from broker (rc={rc}), will auto-reconnect", file=sys.stderr)


def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    elapsed = time.time() - start_time
    print(f"Got message on {msg.topic}: {payload} (elapsed: {elapsed:.1f}s, retained: {msg.retain})")

    if msg.topic in TOPIC_SOUNDS:
        if msg.retain:
            print("  -> Ignoring (retained message replayed on connect/reconnect)")
            return
        should_play = payload == "1" if msg.topic != "mashTun/hops_alarm" else payload not in ("", "0")
        if should_play:
            sound_file = f"{SOUNDS_DIR}/{TOPIC_SOUNDS[msg.topic]}"
            play_sound(sound_file)
    elif msg.topic in REMINDER_SOUNDS:
        # Firmware publishes reminders with retain=false, so this shouldn't
        # normally fire — but guard anyway in case a stray retained message
        # ever ends up on the broker (e.g. from manual testing).
        if msg.retain:
            print("  -> Ignoring (unexpected retained reminder)")
            return
        if payload == "1":
            sound_file = f"{SOUNDS_DIR}/{REMINDER_SOUNDS[msg.topic]}"
            play_sound(sound_file)


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message

client.reconnect_delay_set(min_delay=1, max_delay=30)
client.connect_async(MQTT_HOST, MQTT_PORT, 60)
client.loop_forever(retry_first_connection=True)